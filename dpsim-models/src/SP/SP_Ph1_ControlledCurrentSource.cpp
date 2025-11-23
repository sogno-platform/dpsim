/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_ControlledCurrentSource.h>

using namespace CPS;

SP::Ph1::ControlledCurrentSource::ControlledCurrentSource(String uid, String name,
                                      Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mCurrentRef(mAttributes->createDynamic<Complex>("I_ref")) {
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

SP::Ph1::ControlledCurrentSource::ControlledCurrentSource(String name, Complex current,
                                      Logger::Level logLevel)
    : ControlledCurrentSource(name, logLevel) {
  setParameters(current);
}

void SP::Ph1::ControlledCurrentSource::setParameters(Complex current) {
  **mCurrentRef = current;
  mParametersSet = true;
}

SimPowerComp<Complex>::Ptr SP::Ph1::ControlledCurrentSource::clone(String name) {
  auto copy = ControlledCurrentSource::make(name, mLogLevel);
  copy->setParameters(**mCurrentRef);
  return copy;
}

void SP::Ph1::ControlledCurrentSource::initializeFromNodesAndTerminals(Real frequency) {

  (**mIntfVoltage)(0, 0) = initialSingleVoltage(0) - initialSingleVoltage(1);
  (**mIntfCurrent)(0, 0) = **mCurrentRef;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

void SP::Ph1::ControlledCurrentSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  (**mIntfCurrent)(0, 0) = **mCurrentRef;
}

void SP::Ph1::ControlledCurrentSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mCurrentRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::ControlledCurrentSource::mnaCompPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::ControlledCurrentSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  (**mIntfCurrent)(0, 0) = **mCurrentRef;

  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0),
                           -(**mIntfCurrent)(0, 0));
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1),
                           (**mIntfCurrent)(0, 0));
}

void SP::Ph1::ControlledCurrentSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void SP::Ph1::ControlledCurrentSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
}

void SP::Ph1::ControlledCurrentSource::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
}
