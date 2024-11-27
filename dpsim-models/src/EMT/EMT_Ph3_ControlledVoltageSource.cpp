/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_ControlledVoltageSource.h>

using namespace CPS;

EMT::Ph3::ControlledVoltageSource::ControlledVoltageSource(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel), mVoltageRef(mAttributes->createDynamic<Matrix>("V_ref")) // rms-value, phase-to-phase
{
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::ControlledVoltageSource::setParameters(Matrix voltageRef) {
  **mVoltageRef = voltageRef;

  mParametersSet = true;
}

void EMT::Ph3::ControlledVoltageSource::initializeFromNodesAndTerminals(Real frequency) {
  SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from node voltages ---");
  // TODO: this approach currently overwrites voltage reference set from outside, when not using setParameters
  if (!mParametersSet) {
    // initialize voltage reference as zeroes
    SPDLOG_LOGGER_INFO(mSLog,
                       "\nReference voltage: {:s}"
                       "\nTerminal 0 voltage: {:s}"
                       "\nTerminal 1 voltage: {:s}",
                       Logger::matrixCompToString(**mVoltageRef), Logger::phasorToString(initialSingleVoltage(0)), Logger::phasorToString(initialSingleVoltage(1)));
  } else {
    SPDLOG_LOGGER_INFO(mSLog,
                       "\nInitialization from node voltages omitted (parameter already set)."
                       "\nReference voltage: {:s}",
                       Logger::matrixCompToString(**mVoltageRef));
  }
  SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from node voltages ---");
  mSLog->flush();
}

SimPowerComp<Real>::Ptr EMT::Ph3::ControlledVoltageSource::clone(String name) {
  auto copy = ControlledVoltageSource::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef);
  return copy;
}

void EMT::Ph3::ControlledVoltageSource::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) { updateMatrixNodeIndices(); }

void EMT::Ph3::ControlledVoltageSource::mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) {
  if (terminalNotGrounded(0)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), -1);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(0, 0), -1);

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), -1);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(0, 1), -1);

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), -1);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(0, 2), -1);
  }
  if (terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), 1);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), matrixNodeIndex(1, 0), 1);

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), 1);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), matrixNodeIndex(1, 1), 1);

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), 1);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), matrixNodeIndex(1, 2), 1);
  }
}

void EMT::Ph3::ControlledVoltageSource::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A), (**mIntfVoltage)(0, 0));
  Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B), (**mIntfVoltage)(1, 0));
  Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C), (**mIntfVoltage)(2, 0));
}

void EMT::Ph3::ControlledVoltageSource::updateVoltage(Real time) {
  **mIntfVoltage = **mVoltageRef;

  SPDLOG_LOGGER_DEBUG(mSLog, "\nUpdate Voltage: {:s}", Logger::matrixToString(**mIntfVoltage));
}

void EMT::Ph3::ControlledVoltageSource::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph3::ControlledVoltageSource::mnaCompPreStep(Real time, Int timeStepCount) {
  updateVoltage(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::ControlledVoltageSource::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
                                                                       Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
};

void EMT::Ph3::ControlledVoltageSource::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) { mnaCompUpdateCurrent(**leftVector); }

void EMT::Ph3::ControlledVoltageSource::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::A));
  (**mIntfCurrent)(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::B));
  (**mIntfCurrent)(2, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex(PhaseType::C));
}
