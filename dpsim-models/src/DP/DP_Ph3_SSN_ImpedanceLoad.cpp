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
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mState(mAttributes->create<MatrixComp>("state")),
      mNodalAdmittance(MatrixComp::Zero(3, 3)),
      mInductorAdmittance(MatrixComp::Zero(3, 3)),
      mCapacitorAdmittance(MatrixComp::Zero(3, 3)),
      mCapacitorPreviousVoltage(MatrixComp::Zero(3, 3)),
      mHistoryLoad(MatrixComp::Zero(3, 1)),
      mHistoryShunt(MatrixComp::Zero(3, 1)) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
  **mState = MatrixComp::Zero(12, 1);
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
  (**mState).setZero();
  if (mLoadInductance > 0.) {
    (**mState).block<3, 1>(kLoadInductorVoltage, 0) = voltage;
    (**mState).block<3, 1>(kLoadInductorCurrent, 0) = inductorCurrent;
  }
  if (mLoadCapacitance > 0.) {
    (**mState).block<3, 1>(kLoadCapacitorVoltage, 0) = voltage;
    (**mState).block<3, 1>(kLoadCapacitorCurrent, 0) = capacitorCurrent;
  }
  **mIntfCurrent = -((mLoadConductance + mShuntConductance) * voltage +
                     inductorCurrent + capacitorCurrent);
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompInitialize(Real omega, Real timeStep,
                                                    Attribute<Matrix>::Ptr) {
  updateMatrixNodeIndices();
  mNodalAdmittance = (mLoadConductance + mShuntConductance) *
                     MatrixComp::Identity(3, 3).cast<Complex>();

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
  }
  if (mLoadCapacitance > 0.) {
    const Real capReal = 2. * mLoadCapacitance / timeStep;
    mCapacitorAdmittance =
        Complex(capReal, omega * mLoadCapacitance) * MatrixComp::Identity(3, 3);
    mCapacitorPreviousVoltage = Complex(capReal, -omega * mLoadCapacitance) *
                                MatrixComp::Identity(3, 3);
    mNodalAdmittance += mCapacitorAdmittance;
  }
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(mNodalAdmittance, systemMatrix,
                                       matrixNodeIndex(0), 0,
                                       terminalNotGrounded(0), false, mSLog);
}

void DP::Ph3::SSN::ImpedanceLoad::updateHistory() {
  if (mLoadInductance > 0.)
    mHistoryLoad =
        mInductorAdmittance * (**mState).block<3, 1>(kLoadInductorVoltage, 0) +
        mInductorPreviousCurrent *
            (**mState).block<3, 1>(kLoadInductorCurrent, 0);
  if (mLoadCapacitance > 0.)
    mHistoryShunt = -(**mState).block<3, 1>(kLoadCapacitorCurrent, 0) -
                    mCapacitorPreviousVoltage *
                        (**mState).block<3, 1>(kLoadCapacitorVoltage, 0);
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                           -(mHistoryLoad(phase, 0) + mHistoryShunt(phase, 0)));
}

void DP::Ph3::SSN::ImpedanceLoad::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::addToVectorElement(rightVector, matrixNodeIndex(0, phase),
                             Math::complexFromVectorElement(
                                 **mRightVector, matrixNodeIndex(0, phase)));
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompPreStep(Real, Int) {
  updateHistory();
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

DP::Ph3::SSN::ImpedanceLoad::PhaseVector
DP::Ph3::SSN::ImpedanceLoad::nodeVoltage(const Matrix &leftVector) {
  PhaseVector voltage = PhaseVector::Zero();
  if (terminalNotGrounded(0))
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase, 0) =
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, phase));
  return voltage;
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  **mIntfVoltage = nodeVoltage(leftVector);
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompUpdateCurrent(const Matrix &) {
  updateCurrentFromVoltage(**mIntfVoltage);
}

void DP::Ph3::SSN::ImpedanceLoad::updateCurrentFromVoltage(
    const PhaseVector &voltage) {
  for (UInt phase = 0; phase < 3; ++phase) {
    Complex inductorCurrent = 0.;
    Complex capacitorCurrent = 0.;
    if (mLoadInductance > 0.)
      inductorCurrent = mInductorAdmittance(phase, phase) * voltage(phase) +
                        mHistoryLoad(phase, 0);
    if (mLoadCapacitance > 0.)
      capacitorCurrent = mCapacitorAdmittance(phase, phase) * voltage(phase) +
                         mHistoryShunt(phase, 0);

    (**mIntfCurrent)(phase, 0) =
        -((mLoadConductance + mShuntConductance) * voltage(phase) +
          inductorCurrent + capacitorCurrent);
    if (mLoadInductance > 0.) {
      (**mState)(kLoadInductorVoltage + phase, 0) = voltage(phase);
      (**mState)(kLoadInductorCurrent + phase, 0) = inductorCurrent;
    }
    if (mLoadCapacitance > 0.) {
      (**mState)(kLoadCapacitorVoltage + phase, 0) = voltage(phase);
      (**mState)(kLoadCapacitorCurrent + phase, 0) = capacitorCurrent;
    }
  }
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  const PhaseVector voltage = nodeVoltage(**leftVector);
  **mIntfVoltage = voltage;
  updateCurrentFromVoltage(voltage);
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mState);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph3::SSN::ImpedanceLoad::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mState);
}
