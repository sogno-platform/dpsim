/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_Shunt.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::Shunt::Shunt(String uid, String name, Logger::Level logLevel)
    : Shunt(uid, name, true, logLevel) {}

DP::Ph1::Shunt::Shunt(String uid, String name, Bool useSubcomponents,
                      Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mUseSubcomponents(useSubcomponents),
      mConductance(mAttributes->create<Real>("G")),
      mSusceptance(mAttributes->create<Real>("B")) {

  SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", this->type(), name);
  setTerminalNumber(1);
}

void DP::Ph1::Shunt::setParameters(Real conductance, Real susceptance) {
  **mConductance = conductance;
  **mSusceptance = susceptance;
  SPDLOG_LOGGER_INFO(mSLog, "Conductance={} [S] Susceptance={} [Ohm] ",
                     conductance, susceptance);
  mParametersSet = true;
}

/// MNA Section
void DP::Ph1::Shunt::initializeParentFromNodesAndTerminals(Real frequency) {

  // Static calculation
  Real omega = 2. * PI * frequency;
  Complex admittance = Complex(**mConductance, **mSusceptance);
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) * admittance;

  if (mUseSubcomponents && **mConductance > 0) {
    mSubResistor =
        std::make_shared<DP::Ph1::Resistor>(**mName + "_Res", mLogLevel);
    mSubResistor->connect(SimNode::List{SimNode::GND, mTerminals[0]->node()});
    mSubResistor->setParameters(1. / **mConductance);
    mSubResistor->initialize(mFrequencies);
    mSubResistor->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  }

  if (**mSusceptance > 0) {
    mDirectCapacitance = **mSusceptance / omega;
    if (mUseSubcomponents) {
      mSubCapacitor =
          std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap", mLogLevel);
      mSubCapacitor->setParameters(mDirectCapacitance);
      mSubCapacitor->connect(
          SimNode::List{SimNode::GND, mTerminals[0]->node()});
      mSubCapacitor->initialize(mFrequencies);
      mSubCapacitor->initializeFromNodesAndTerminals(frequency);
      addMNASubComponent(mSubCapacitor,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                         MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
    }
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)));
}

void DP::Ph1::Shunt::mnaParentInitialize(Real omega, Real timeStep,
                                         Attribute<Matrix>::Ptr leftVector) {
  mDirectEquivCond = MatrixComp::Zero(mNumFreqs, 1);
  mDirectPrevVoltCoeff = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapCurrent = MatrixComp::Zero(mNumFreqs, 1);
  mDirectHistory = MatrixComp::Zero(mNumFreqs, 1);
  if (mUseSubcomponents || mDirectCapacitance <= 0.)
    return;

  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Real capacitiveConductance = 2. * mDirectCapacitance / timeStep;
    const Real harmonicSusceptance =
        2. * PI * mFrequencies(freq, 0) * mDirectCapacitance;
    mDirectEquivCond(freq, 0) =
        Complex(capacitiveConductance, harmonicSusceptance);
    mDirectPrevVoltCoeff(freq, 0) =
        Complex(capacitiveConductance, -harmonicSusceptance);
    if (freq == 0)
      mDirectCapCurrent(freq, 0) =
          Complex(0., 2. * PI * mFrequencies(freq, 0) * mDirectCapacitance) *
          (**mIntfVoltage)(0, 0);
  }
}

void DP::Ph1::Shunt::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Complex admittance =
        Complex(**mConductance, 0.) + mDirectEquivCond(freq, 0);
    MNAStampUtils::stampAdmittance(admittance, systemMatrix, matrixNodeIndex(0),
                                   0, terminalNotGrounded(0), false, mSLog,
                                   mNumFreqs, freq);
  }
}

void DP::Ph1::Shunt::mnaParentApplyRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents || mDirectCapacitance <= 0.)
    return;
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Complex history =
        -mDirectCapCurrent(freq, 0) -
        mDirectPrevVoltCoeff(freq, 0) * (**mIntfVoltage)(0, freq);
    mDirectHistory(freq, 0) = history;
    if (terminalNotGrounded(0))
      Math::setVectorElement(rightVector, matrixNodeIndex(0), -history,
                             mNumFreqs, freq);
  }
}

void DP::Ph1::Shunt::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  if (!mUseSubcomponents) {
    prevStepDependencies.push_back(mIntfVoltage);
    prevStepDependencies.push_back(mIntfCurrent);
  }
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::Shunt::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::Shunt::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::Shunt::mnaParentPostStep(Real time, Int timeStepCount,
                                       Attribute<Matrix>::Ptr &leftVector) {
  this->mnaUpdateVoltage(**leftVector);
  this->mnaUpdateCurrent(**leftVector);
}

void DP::Ph1::Shunt::mnaCompUpdateVoltage(const Matrix &leftVector) {
  for (UInt freq = 0; freq < mNumFreqs; ++freq)
    (**mIntfVoltage)(0, freq) = Math::complexFromVectorElement(
        leftVector, matrixNodeIndex(0), mNumFreqs, freq);
}

void DP::Ph1::Shunt::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (!mUseSubcomponents) {
    for (UInt freq = 0; freq < mNumFreqs; ++freq) {
      const Complex voltage = (**mIntfVoltage)(0, freq);
      const Complex capCurrent =
          mDirectCapacitance > 0.
              ? mDirectEquivCond(freq, 0) * voltage + mDirectHistory(freq, 0)
              : Complex(0., 0.);
      mDirectCapCurrent(freq, 0) = capCurrent;
      (**mIntfCurrent)(0, freq) =
          Complex(**mConductance, 0.) * voltage + capCurrent;
    }
    return;
  }

  (**mIntfCurrent)(0, 0) = 0;
  if (**mConductance > 0)
    (**mIntfCurrent)(0, 0) += mSubResistor->intfCurrent()(0, 0);
  if (**mSusceptance > 0)
    (**mIntfCurrent)(0, 0) += mSubCapacitor->intfCurrent()(0, 0);
}
