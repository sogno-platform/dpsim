/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_ControlledCurrentSource.h>

using namespace CPS;

EMT::Ph3::ControlledCurrentSource::ControlledCurrentSource(
    String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mCurrentRef(mAttributes->create<Matrix>("I_ref")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(0);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}
SimPowerComp<Real>::Ptr EMT::Ph3::ControlledCurrentSource::clone(String name) {
  auto copy = ControlledCurrentSource::make(name, mLogLevel);
  copy->setParameters(attributeTyped<Matrix>("I_ref")->get());
  return copy;
}

void EMT::Ph3::ControlledCurrentSource::setParameters(Matrix currentRef) {
  **mCurrentRef = currentRef;
  SPDLOG_LOGGER_INFO(
      mSLog, "\nCurrent reference phasor [I]: {:s}",
              Logger::matrixCompToString(currentRef)
  );
  mParametersSet = true;
}

void EMT::Ph3::ControlledCurrentSource::initializeFromNodesAndTerminals(
    Real frequency) {
  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminal ---");
  if (!mParametersSet) {
    **mCurrentRef = Matrix::Zero(3, 1);

    SPDLOG_LOGGER_INFO(mSLog,
                       "\nTerminal 0 voltage: {:s}"
                       "\nTerminal 1 voltage: {:s}"
                       "\nTerminal 0 power: {:s}"
                       "\nTerminal 1 power: {:s}",
                       Logger::phasorToString(initialSingleVoltage(0)),
                       Logger::phasorToString(initialSingleVoltage(1)),
                       Logger::complexToString(terminal(0)->singlePower()),
                       Logger::complexToString(terminal(1)->singlePower()));
  } else {
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\nInitialization from node voltages and terminal omitted (parameter "
        "already set)."
        "\nReference voltage: {:s}",
        Logger::matrixToString(attributeTyped<Matrix>("I_ref")->get()));
  }
  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminal ---");
  mSLog->flush();
}

void EMT::Ph3::ControlledCurrentSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
}

void EMT::Ph3::ControlledCurrentSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(1)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0),
                           -(**mIntfCurrent)(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 1),
                           -(**mIntfCurrent)(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 2),
                           -(**mIntfCurrent)(2, 0));
  }
  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                           (**mIntfCurrent)(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                           (**mIntfCurrent)(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                           (**mIntfCurrent)(2, 0));
  }
}

void EMT::Ph3::ControlledCurrentSource::updateCurrent(Real time) {
  **mIntfCurrent = **mCurrentRef;

  SPDLOG_LOGGER_DEBUG(mSLog, "\nUpdate current: {:s}",
                      Logger::matrixToString(**mIntfCurrent));
}

void EMT::Ph3::ControlledCurrentSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mCurrentRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph3::ControlledCurrentSource::mnaCompPreStep(Real time,
                                                       Int timeStepCount) {
  updateCurrent(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::ControlledCurrentSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
};

void EMT::Ph3::ControlledCurrentSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::ControlledCurrentSource::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}
