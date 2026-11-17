/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph3_SSN_ImpedanceLoad.h>
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

EMT::Ph3::SSN::ImpedanceLoad::ImpedanceLoad(String uid, String name,
                                            Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 3, 3, logLevel), mStateValue(&mX->get()),
      mNodalAdmittance(Matrix::Zero(3, 3)),
      mInductorAdmittance(Matrix::Zero(3, 3)),
      mCapacitorAdmittance(Matrix::Zero(3, 3)),
      mHistoryLoad(Matrix::Zero(3, 1)), mHistoryShunt(Matrix::Zero(3, 1)) {
  setTerminalNumber(1);
  *mStateValue = Matrix::Zero(12, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::ImpedanceLoad::clone(String copySuffix) {
  auto copy = SharedFactory<ImpedanceLoad>::make(copySuffix, mLogLevel);
  copy->setParameters(mActivePower, mReactivePower, mNominalVoltage,
                      mShuntConductance, mShuntSusceptance);
  return copy;
}

void EMT::Ph3::SSN::ImpedanceLoad::setParameters(Real activePower,
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
  mTotalConductance = mLoadConductance + shuntConductance;
  EMT::SSNComp::setParameters(Matrix::Zero(12, 12), Matrix::Zero(12, 3),
                              Matrix::Zero(3, 12), Matrix::Zero(3, 3));
}

void EMT::Ph3::SSN::ImpedanceLoad::initializeFromNodesAndTerminals(
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

  MatrixComp voltagePhasor = MatrixComp::Zero(3, 1);
  voltagePhasor(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  voltagePhasor(1, 0) = voltagePhasor(0, 0) * SHIFT_TO_PHASE_B;
  voltagePhasor(2, 0) = voltagePhasor(0, 0) * SHIFT_TO_PHASE_C;
  MatrixComp inductorCurrentPhasor = MatrixComp::Zero(3, 1);
  MatrixComp capacitorCurrentPhasor = MatrixComp::Zero(3, 1);
  if (mLoadInductance > 0.)
    inductorCurrentPhasor =
        Complex(0., -1. / (omega * mLoadInductance)) * voltagePhasor;
  if (mLoadCapacitance > 0.)
    capacitorCurrentPhasor =
        Complex(0., omega * mLoadCapacitance) * voltagePhasor;

  const Matrix voltage = voltagePhasor.real();
  const Matrix inductorCurrent = inductorCurrentPhasor.real();
  const Matrix capacitorCurrent = capacitorCurrentPhasor.real();
  *mIntfVoltageValue = voltage;
  mStateValue->setZero();
  if (mLoadInductance > 0.) {
    mStateValue->block<3, 1>(kLoadInductorVoltage, 0) = voltage;
    mStateValue->block<3, 1>(kLoadInductorCurrent, 0) = inductorCurrent;
  }
  if (mLoadCapacitance > 0.) {
    mStateValue->block<3, 1>(kLoadCapacitorVoltage, 0) = voltage;
    mStateValue->block<3, 1>(kLoadCapacitorCurrent, 0) = capacitorCurrent;
  }
  *mIntfCurrentValue =
      mTotalConductance * voltage + inductorCurrent + capacitorCurrent;
}

MatrixComp EMT::Ph3::SSN::ImpedanceLoad::buildInitialInputFromNodes(Real) {
  MatrixComp voltage = MatrixComp::Zero(3, 1);
  const Complex phaseA = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  voltage(0, 0) = phaseA;
  voltage(1, 0) = phaseA * SHIFT_TO_PHASE_B;
  voltage(2, 0) = phaseA * SHIFT_TO_PHASE_C;
  return voltage;
}

void EMT::Ph3::SSN::ImpedanceLoad::recomputeDiscreteModel() {
  mNodalAdmittance = mTotalConductance * Matrix::Identity(3, 3);
  if (mLoadInductance > 0.) {
    mInductorAdmittanceScalar = mTimeStep / (2. * mLoadInductance);
    mInductorAdmittance = mInductorAdmittanceScalar * Matrix::Identity(3, 3);
    mNodalAdmittance += mInductorAdmittance;
  } else {
    mInductorAdmittanceScalar = 0.;
    mInductorAdmittance.setZero();
  }
  if (mLoadCapacitance > 0.) {
    mCapacitorAdmittanceScalar = 2. * mLoadCapacitance / mTimeStep;
    mCapacitorAdmittance = mCapacitorAdmittanceScalar * Matrix::Identity(3, 3);
    mNodalAdmittance += mCapacitorAdmittance;
  } else {
    mCapacitorAdmittanceScalar = 0.;
    mCapacitorAdmittance.setZero();
  }
  mW = mNodalAdmittance;
  mdA.setZero(12, 12);
  mdB.setZero(12, 3);
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductanceMatrix(mW, systemMatrix, matrixNodeIndex(0), 0,
                                        terminalNotGrounded(0), false, mSLog);
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                           -mYHist(phase, 0));
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  mIntfVoltageValue->setZero();
  if (terminalNotGrounded(0))
    for (UInt phase = 0; phase < 3; ++phase)
      (*mIntfVoltageValue)(phase, 0) =
          Math::realFromVectorElement(leftVector, matrixNodeIndex(0, phase));
}

void EMT::Ph3::SSN::ImpedanceLoad::calculateHistoryVector(
    Matrix &history) const {
  mHistoryLoad.setZero();
  mHistoryShunt.setZero();
  if (mLoadInductance > 0.)
    for (UInt phase = 0; phase < 3; ++phase)
      mHistoryLoad(phase, 0) =
          mInductorAdmittanceScalar * (*mStateValue)(phase, 0) +
          (*mStateValue)(3 + phase, 0);
  if (mLoadCapacitance > 0.)
    for (UInt phase = 0; phase < 3; ++phase)
      mHistoryShunt(phase, 0) =
          -(*mStateValue)(9 + phase, 0) -
          mCapacitorAdmittanceScalar * (*mStateValue)(6 + phase, 0);
  history = mHistoryLoad + mHistoryShunt;
}

void EMT::Ph3::SSN::ImpedanceLoad::updateState(const Matrix &,
                                               const Matrix &voltage) {
  if (mLoadInductance > 0.) {
    const PhaseVector current =
        mInductorAdmittanceScalar * voltage + mHistoryLoad;
    mStateValue->block<3, 1>(0, 0) = voltage;
    mStateValue->block<3, 1>(3, 0) = current;
  }
  if (mLoadCapacitance > 0.) {
    const PhaseVector current =
        mCapacitorAdmittanceScalar * voltage + mHistoryShunt;
    mStateValue->block<3, 1>(6, 0) = voltage;
    mStateValue->block<3, 1>(9, 0) = current;
  }
}
