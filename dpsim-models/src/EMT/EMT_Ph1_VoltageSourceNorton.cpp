/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_VoltageSourceNorton.h>

using namespace CPS;

EMT::Ph1::VoltageSourceNorton::VoltageSourceNorton(String uid, String name,
                                                   Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      Base::Ph1::VoltageSource(mAttributes),
      mResistance(mAttributes->create<Real>("R")) {
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph1::VoltageSourceNorton::clone(String name) {
  auto copy = VoltageSourceNorton::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef, **mSrcFreq, **mResistance);
  return copy;
}

void EMT::Ph1::VoltageSourceNorton::setParameters(Complex voltage, Real srcFreq,
                                                  Real resistance) {
  Base::Ph1::VoltageSource::setParameters(voltage, srcFreq);

  **mResistance = resistance;
  mConductance = 1. / **mResistance;

  mParametersSet = true;
}

void EMT::Ph1::VoltageSourceNorton::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

  (**mIntfVoltage)(0, 0) = (**mVoltageRef).real();
}

void EMT::Ph1::VoltageSourceNorton::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Apply matrix stamp for equivalent resistance
  if (terminalNotGrounded(0))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(0), mConductance);
  if (terminalNotGrounded(1))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(1), mConductance);
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(1), -mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(0), -mConductance);
  }
}

void EMT::Ph1::VoltageSourceNorton::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // Apply matrix stamp for equivalent current source
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0), -mEquivCurrent);
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1), mEquivCurrent);
}

void EMT::Ph1::VoltageSourceNorton::updateState(Real time) {
  // Check if set source was called
  if (Math::abs(**mVoltageRef) > 0)
    (**mIntfVoltage)(0, 0) =
        Math::abs(**mVoltageRef) *
        cos(2. * PI * **mSrcFreq * time + Math::phase(**mVoltageRef));

  mEquivCurrent = (**mIntfVoltage)(0, 0) / **mResistance;
}

void EMT::Ph1::VoltageSourceNorton::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph1::VoltageSourceNorton::mnaCompPreStep(Real time,
                                                   Int timeStepCount) {
  updateState(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::VoltageSourceNorton::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph1::VoltageSourceNorton::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::VoltageSourceNorton::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // Calculate v1 - v0
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::VoltageSourceNorton::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  // TODO: verify signs
  (**mIntfCurrent)(0, 0) =
      mEquivCurrent - (**mIntfVoltage)(0, 0) / **mResistance;
}

void EMT::Ph1::VoltageSourceNorton::setVoltageRef(Complex voltage) const {
  **mVoltageRef = voltage;
}
