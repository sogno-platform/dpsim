/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_Inductor.h>

using namespace CPS;

EMT::Ph1::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      Base::Ph1::Inductor(mAttributes) {
  mEquivCurrent = 0;
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  setTerminalNumber(2);
}

SimPowerComp<Real>::Ptr EMT::Ph1::Inductor::clone(String name) {
  auto copy = Inductor::make(name, mLogLevel);
  copy->setParameters(**mInductance);
  return copy;
}

void EMT::Ph1::Inductor::initializeFromNodesAndTerminals(Real frequency) {

  Real omega = 2 * PI * frequency;
  Complex impedance = {0, omega * **mInductance};
  Complex voltage =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  (**mIntfVoltage)(0, 0) = voltage.real();
  (**mIntfCurrent)(0, 0) = (voltage / impedance).real();

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:f}"
                     "\nCurrent: {:f}"
                     "\nTerminal 0 voltage: {:f}"
                     "\nTerminal 1 voltage: {:f}"
                     "\n--- Initialization from powerflow finished ---",
                     (**mIntfVoltage)(0, 0), (**mIntfCurrent)(0, 0),
                     (RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)).real(),
                     (RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)).real());
}

void EMT::Ph1::Inductor::mnaCompInitialize(Real omega, Real timeStep,
                                           Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

  mEquivCond = timeStep / (2.0 * **mInductance);
  // Update internal state
  mEquivCurrent = mEquivCond * (**mIntfVoltage)(0, 0) + (**mIntfCurrent)(0, 0);
}

void EMT::Ph1::Inductor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductance(mEquivCond, systemMatrix, matrixNodeIndex(0),
                                  matrixNodeIndex(1), terminalNotGrounded(0),
                                  terminalNotGrounded(1));
}

void EMT::Ph1::Inductor::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  // Update internal state
  mEquivCurrent = mEquivCond * (**mIntfVoltage)(0, 0) + (**mIntfCurrent)(0, 0);
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent);
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent);
}

void EMT::Ph1::Inductor::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // actually depends on L, but then we'd have to modify the system matrix anyway
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
}

void EMT::Ph1::Inductor::mnaCompPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::Inductor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::Inductor::mnaCompPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::Inductor::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::Inductor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mEquivCond * (**mIntfVoltage)(0, 0) + mEquivCurrent;
}
