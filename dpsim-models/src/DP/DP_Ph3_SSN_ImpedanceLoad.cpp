/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph3_SSN_ImpedanceLoad.h>
#include <dpsim-models/MNAStampUtils.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace CPS;

namespace {
constexpr Int kLoadInductorVoltage = 0;
constexpr Int kLoadInductorCurrent = 3;
constexpr Int kLoadCapacitorVoltage = 6;
constexpr Int kLoadCapacitorCurrent = 9;
} // namespace

DP::Ph3::SSN::ImpedanceLoad::ImpedanceLoad(String uid, String name,
                                           Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 3, 3, logLevel),
      mNodalAdmittance(MatrixComp::Zero(3, 3)),
      mInductorAdmittance(MatrixComp::Zero(3, 3)),
      mCapacitorAdmittance(MatrixComp::Zero(3, 3)),
      mCapacitorPreviousVoltage(MatrixComp::Zero(3, 3)),
      mHistoryLoad(MatrixComp::Zero(3, 1)),
      mHistoryShunt(MatrixComp::Zero(3, 1)) {
  setTerminalNumber(1);
  **mX = MatrixComp::Zero(12, 1);
}

SimPowerComp<Complex>::Ptr
DP::Ph3::SSN::ImpedanceLoad::clone(String copySuffix) {
  auto copy = SharedFactory<ImpedanceLoad>::make(copySuffix, mLogLevel);
  copy->setParameters(mActivePower, mReactivePower, mNominalVoltage,
                      mShuntConductance, mShuntSusceptance);
  return copy;
}

void DP::Ph3::SSN::ImpedanceLoad::setParameters(Real activePower,
                                                Real reactivePower,
                                                Real nominalVoltage,
                                                Real shuntConductance,
                                                Real shuntSusceptance) {
  if (!std::isfinite(activePower) || !std::isfinite(reactivePower) ||
      !std::isfinite(nominalVoltage) || !std::isfinite(shuntConductance) ||
      !std::isfinite(shuntSusceptance) ||
      ((activePower != 0. || reactivePower != 0.) && nominalVoltage <= 0.))
    throw std::invalid_argument("Invalid impedance-load parameters.");
  mActivePower = activePower;
  mReactivePower = reactivePower;
  mNominalVoltage = nominalVoltage;
  mShuntConductance = shuntConductance;
  mShuntSusceptance = shuntSusceptance;
  mLoadConductance = nominalVoltage > 0.
                         ? activePower / (nominalVoltage * nominalVoltage)
                         : 0.;
  DP::SSNComp::setParameters(Matrix::Zero(12, 12), Matrix::Zero(12, 3),
                             Matrix::Zero(3, 12), Matrix::Zero(3, 3));
}

void DP::Ph3::SSN::ImpedanceLoad::initializeFromNodesAndTerminals(
    Real frequency) {
  const Real omega = 2. * PI * frequency;
  const Real loadSusceptance =
      mNominalVoltage > 0.
          ? mReactivePower / (mNominalVoltage * mNominalVoltage)
          : 0.;
  const Real inductiveSusceptance =
      std::max(loadSusceptance, 0.) + std::max(-mShuntSusceptance, 0.);
  const Real capacitiveSusceptance =
      std::max(-loadSusceptance, 0.) + std::max(mShuntSusceptance, 0.);
  mLoadInductance =
      inductiveSusceptance > 0. ? 1. / (omega * inductiveSusceptance) : 0.;
  mLoadCapacitance = capacitiveSusceptance / omega;
  MatrixComp voltage = MatrixComp::Zero(3, 1);
  voltage(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  voltage(1, 0) = voltage(0, 0) * SHIFT_TO_PHASE_B;
  voltage(2, 0) = voltage(0, 0) * SHIFT_TO_PHASE_C;
  MatrixComp inductorCurrent = MatrixComp::Zero(3, 1);
  MatrixComp capacitorCurrent = MatrixComp::Zero(3, 1);
  if (mLoadInductance > 0.)
    inductorCurrent = Complex(0., -1. / (omega * mLoadInductance)) * voltage;
  if (mLoadCapacitance > 0.)
    capacitorCurrent = Complex(0., omega * mLoadCapacitance) * voltage;

  **mIntfVoltage = voltage;
  (**mX).setZero();
  if (mLoadInductance > 0.) {
    (**mX).block<3, 1>(kLoadInductorVoltage, 0) = voltage;
    (**mX).block<3, 1>(kLoadInductorCurrent, 0) = inductorCurrent;
  }
  if (mLoadCapacitance > 0.) {
    (**mX).block<3, 1>(kLoadCapacitorVoltage, 0) = voltage;
    (**mX).block<3, 1>(kLoadCapacitorCurrent, 0) = capacitorCurrent;
  }
  **mIntfCurrent = (mLoadConductance + mShuntConductance) * voltage +
                   inductorCurrent + capacitorCurrent;
}

MatrixComp DP::Ph3::SSN::ImpedanceLoad::buildInitialInputFromNodes(Real) {
  MatrixComp voltage = MatrixComp::Zero(3, 1);
  const Complex phaseA = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  voltage(0, 0) = phaseA;
  voltage(1, 0) = phaseA * SHIFT_TO_PHASE_B;
  voltage(2, 0) = phaseA * SHIFT_TO_PHASE_C;
  return voltage;
}

void DP::Ph3::SSN::ImpedanceLoad::recomputeDiscreteModel(Real omega) {
  const Real timeStep = mTimeStep;
  mNodalAdmittance =
      (mLoadConductance + mShuntConductance) * MatrixComp::Identity(3, 3);
  const Real inductiveSusceptance =
      (mNominalVoltage > 0.
           ? std::max(mReactivePower / (mNominalVoltage * mNominalVoltage), 0.)
           : 0.) +
      std::max(-mShuntSusceptance, 0.);
  const Real capacitiveSusceptance =
      (mNominalVoltage > 0.
           ? std::max(-mReactivePower / (mNominalVoltage * mNominalVoltage), 0.)
           : 0.) +
      std::max(mShuntSusceptance, 0.);
  mLoadInductance =
      inductiveSusceptance > 0. ? 1. / (omega * inductiveSusceptance) : 0.;
  mLoadCapacitance = capacitiveSusceptance / omega;
  if (mLoadInductance > 0.) {
    const Real b = timeStep * omega / 2.;
    const Complex y(timeStep / (2. * mLoadInductance) / (1. + b * b),
                    -timeStep / (2. * mLoadInductance) * b / (1. + b * b));
    mInductorAdmittance = y * MatrixComp::Identity(3, 3);
    mInductorPreviousCurrent =
        Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
    mNodalAdmittance += mInductorAdmittance;
  } else {
    mInductorAdmittance.setZero();
  }
  if (mLoadCapacitance > 0.) {
    const Real capReal = 2. * mLoadCapacitance / timeStep;
    mCapacitorAdmittance =
        Complex(capReal, omega * mLoadCapacitance) * MatrixComp::Identity(3, 3);
    mCapacitorPreviousVoltage = Complex(capReal, -omega * mLoadCapacitance) *
                                MatrixComp::Identity(3, 3);
    mNodalAdmittance += mCapacitorAdmittance;
  } else {
    mCapacitorAdmittance.setZero();
    mCapacitorPreviousVoltage.setZero();
  }
  mW = mNodalAdmittance;
  mDiscreteA.setZero(12, 12);
  mDiscreteB.setZero(12, 3);
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(mW, systemMatrix, matrixNodeIndex(0), 0,
                                       terminalNotGrounded(0), false, mSLog);
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                           -mYHist(phase, 0));
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  (**mIntfVoltage).setZero();
  if (terminalNotGrounded(0))
    for (UInt phase = 0; phase < 3; ++phase)
      (**mIntfVoltage)(phase, 0) =
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, phase));
}

MatrixComp DP::Ph3::SSN::ImpedanceLoad::calculateHistoryVector() const {
  mHistoryLoad.setZero();
  mHistoryShunt.setZero();
  if (mLoadInductance > 0.)
    mHistoryLoad =
        mInductorAdmittance * (**mX).block<3, 1>(kLoadInductorVoltage, 0) +
        mInductorPreviousCurrent * (**mX).block<3, 1>(kLoadInductorCurrent, 0);
  if (mLoadCapacitance > 0.)
    mHistoryShunt = -(**mX).block<3, 1>(kLoadCapacitorCurrent, 0) -
                    mCapacitorPreviousVoltage *
                        (**mX).block<3, 1>(kLoadCapacitorVoltage, 0);
  return mHistoryLoad + mHistoryShunt;
}

void DP::Ph3::SSN::ImpedanceLoad::updateHistoryVector(MatrixComp &history) {
  history = calculateHistoryVector();
}

void DP::Ph3::SSN::ImpedanceLoad::updateState(const MatrixComp &,
                                              const MatrixComp &voltage) {
  if (mLoadInductance > 0.) {
    const MatrixComp current = mInductorAdmittance * voltage + mHistoryLoad;
    (**mX).block<3, 1>(kLoadInductorVoltage, 0) = voltage;
    (**mX).block<3, 1>(kLoadInductorCurrent, 0) = current;
  }
  if (mLoadCapacitance > 0.) {
    const MatrixComp current = mCapacitorAdmittance * voltage + mHistoryShunt;
    (**mX).block<3, 1>(kLoadCapacitorVoltage, 0) = voltage;
    (**mX).block<3, 1>(kLoadCapacitorCurrent, 0) = current;
  }
}
