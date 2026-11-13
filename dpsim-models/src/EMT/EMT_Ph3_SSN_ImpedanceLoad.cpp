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
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mState(mAttributes->create<Matrix>("state")), mStateValue(&mState->get()),
      mNodalAdmittance(Matrix::Zero(3, 3)),
      mInductorAdmittance(Matrix::Zero(3, 3)),
      mCapacitorAdmittance(Matrix::Zero(3, 3)),
      mHistoryLoad(Matrix::Zero(3, 1)), mHistoryShunt(Matrix::Zero(3, 1)) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(1);
  *mIntfVoltageValue = Matrix::Zero(3, 1);
  *mIntfCurrentValue = Matrix::Zero(3, 1);
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
      -(mTotalConductance * voltage + inductorCurrent + capacitorCurrent);
}

void EMT::Ph3::SSN::ImpedanceLoad::exportFusedModel(LinearSSNModel &model) {
  model = LinearSSNModel{};
  model.kind = LinearSSNModel::Kind::ImpedanceLoad;
  model.terminalCount = 1;
  model.grounded[0] = !terminalNotGrounded(0);
  for (UInt phase = 0; phase < 3; ++phase)
    model.nodeIndices[phase] = matrixNodeIndex(0, phase);
  for (UInt phase = 0; phase < 3; ++phase)
    model.nodeNames[phase] = node(0)->name();
  model.state = *mStateValue;
  model.portAdmittance = mNodalAdmittance;
  model.totalConductance = mTotalConductance;
  model.inductorAdmittanceScalar = mInductorAdmittanceScalar;
  model.capacitorAdmittanceScalar = mCapacitorAdmittanceScalar;
  model.hasLoadInductance = mLoadInductance > 0.;
  model.hasLoadCapacitance = mLoadCapacitance > 0.;
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompInitialize(Real, Real timeStep,
                                                     Attribute<Matrix>::Ptr) {
  updateMatrixNodeIndices();
  mNodalAdmittance = mTotalConductance * Matrix::Identity(3, 3);
  if (mLoadInductance > 0.) {
    mInductorAdmittanceScalar = timeStep / (2. * mLoadInductance);
    mInductorAdmittance = mInductorAdmittanceScalar * Matrix::Identity(3, 3);
    mNodalAdmittance += mInductorAdmittance;
  } else {
    mInductorAdmittanceScalar = 0.;
  }
  if (mLoadCapacitance > 0.) {
    mCapacitorAdmittanceScalar = 2. * mLoadCapacitance / timeStep;
    mCapacitorAdmittance = mCapacitorAdmittanceScalar * Matrix::Identity(3, 3);
    mNodalAdmittance += mCapacitorAdmittance;
  } else {
    mCapacitorAdmittanceScalar = 0.;
  }
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductanceMatrix(mNodalAdmittance, systemMatrix,
                                        matrixNodeIndex(0), 0,
                                        terminalNotGrounded(0), false, mSLog);
}

void EMT::Ph3::SSN::ImpedanceLoad::updateHistory() {
  if (mLoadInductance > 0.) {
    for (UInt phase = 0; phase < 3; ++phase)
      mHistoryLoad(phase, 0) =
          mInductorAdmittanceScalar *
              (*mStateValue)(kLoadInductorVoltage + phase, 0) +
          (*mStateValue)(kLoadInductorCurrent + phase, 0);
  }
  if (mLoadCapacitance > 0.) {
    for (UInt phase = 0; phase < 3; ++phase)
      mHistoryShunt(phase, 0) =
          -(*mStateValue)(kLoadCapacitorCurrent + phase, 0) -
          mCapacitorAdmittanceScalar *
              (*mStateValue)(kLoadCapacitorVoltage + phase, 0);
  }
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                           -(mHistoryLoad(phase, 0) + mHistoryShunt(phase, 0)));
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::addToVectorElement(
        rightVector, matrixNodeIndex(0, phase),
        -(mHistoryLoad(phase, 0) + mHistoryShunt(phase, 0)));
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompPreStep(Real, Int) {
  updateHistory();
}

EMT::Ph3::SSN::ImpedanceLoad::PhaseVector
EMT::Ph3::SSN::ImpedanceLoad::nodeVoltage(const Matrix &leftVector) {
  PhaseVector voltage = PhaseVector::Zero();
  if (terminalNotGrounded(0))
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase, 0) =
          Math::realFromVectorElement(leftVector, matrixNodeIndex(0, phase));
  return voltage;
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  *mIntfVoltageValue = nodeVoltage(leftVector);
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompUpdateCurrent(const Matrix &) {
  updateCurrentFromVoltage(*mIntfVoltageValue);
}

void EMT::Ph3::SSN::ImpedanceLoad::updateCurrentFromVoltage(
    const PhaseVector &voltage) {
  PhaseVector inductorCurrent = PhaseVector::Zero();
  PhaseVector capacitorCurrent = PhaseVector::Zero();
  if (mLoadInductance > 0.)
    for (UInt phase = 0; phase < 3; ++phase)
      inductorCurrent(phase) =
          mInductorAdmittanceScalar * voltage(phase) + mHistoryLoad(phase, 0);
  if (mLoadCapacitance > 0.)
    for (UInt phase = 0; phase < 3; ++phase)
      capacitorCurrent(phase) =
          mCapacitorAdmittanceScalar * voltage(phase) + mHistoryShunt(phase, 0);

  *mIntfCurrentValue =
      -(mTotalConductance * voltage + inductorCurrent + capacitorCurrent);
  if (mLoadInductance > 0.) {
    mStateValue->block<3, 1>(kLoadInductorVoltage, 0) = voltage;
    mStateValue->block<3, 1>(kLoadInductorCurrent, 0) = inductorCurrent;
  }
  if (mLoadCapacitance > 0.) {
    mStateValue->block<3, 1>(kLoadCapacitorVoltage, 0) = voltage;
    mStateValue->block<3, 1>(kLoadCapacitorCurrent, 0) = capacitorCurrent;
  }
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  const PhaseVector voltage = nodeVoltage(**leftVector);
  *mIntfVoltageValue = voltage;
  updateCurrentFromVoltage(voltage);
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mState);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::SSN::ImpedanceLoad::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mState);
}
