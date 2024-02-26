/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_VoltageSourceNorton.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::VoltageSourceNorton::VoltageSourceNorton(String uid, String name,
                                                   Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      Base::Ph1::VoltageSource(mAttributes),
      mResistance(mAttributes->create<Real>("R")) {
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::VoltageSourceNorton::setParameters(Complex voltageRef,
                                                  Real srcFreq,
                                                  Real resistance) {

  Base::Ph1::VoltageSource::setParameters(voltageRef, srcFreq);
  **mResistance = resistance;
  mConductance = 1. / **mResistance;

  mParametersSet = true;
}

void EMT::Ph3::VoltageSourceNorton::setVoltageRef(Complex voltage) const {
  **mVoltageRef = voltage;
}

SimPowerComp<Real>::Ptr EMT::Ph3::VoltageSourceNorton::clone(String name) {
  auto copy = VoltageSourceNorton::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef, **mSrcFreq, **mResistance);
  return copy;
}

void EMT::Ph3::VoltageSourceNorton::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  Complex voltageRef = mVoltageRef->get();
  (**mIntfVoltage)(0, 0) = voltageRef.real() * cos(Math::phase(voltageRef));
  (**mIntfVoltage)(1, 0) =
      voltageRef.real() * cos(Math::phase(voltageRef) - 2. / 3. * M_PI);
  (**mIntfVoltage)(2, 0) =
      voltageRef.real() * cos(Math::phase(voltageRef) + 2. / 3. * M_PI);
}

void EMT::Ph3::VoltageSourceNorton::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Apply matrix stamp for equivalent resistance
  if (terminalNotGrounded(0)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(0, 0), mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1),
                             matrixNodeIndex(0, 1), mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2),
                             matrixNodeIndex(0, 2), mConductance);
  }
  if (terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(1, 0), mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1),
                             matrixNodeIndex(1, 1), mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2),
                             matrixNodeIndex(1, 2), mConductance);
  }
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(1, 0), -mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(0, 0), -mConductance);

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1),
                             matrixNodeIndex(1, 1), -mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1),
                             matrixNodeIndex(0, 1), -mConductance);

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2),
                             matrixNodeIndex(1, 2), -mConductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2),
                             matrixNodeIndex(0, 2), -mConductance);
  }
}

void EMT::Ph3::VoltageSourceNorton::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // Apply matrix stamp for equivalent current source
  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                           -mEquivCurrent(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                           -mEquivCurrent(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                           -mEquivCurrent(2, 0));
  }
  if (terminalNotGrounded(1)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0),
                           mEquivCurrent(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 1),
                           mEquivCurrent(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 2),
                           mEquivCurrent(2, 0));
  }
}

void EMT::Ph3::VoltageSourceNorton::updateState(Real time) {
  // Check if set source was called
  if (Math::abs(**mVoltageRef) > 0) {
    (**mIntfVoltage)(0, 0) =
        Math::abs(**mVoltageRef) *
        cos(2. * PI * **mSrcFreq * time + Math::phase(**mVoltageRef));
    (**mIntfVoltage)(1, 0) = Math::abs(**mVoltageRef) *
                             cos(2. * PI * **mSrcFreq * time +
                                 Math::phase(**mVoltageRef) - 2. / 3. * M_PI);
    (**mIntfVoltage)(2, 0) = Math::abs(**mVoltageRef) *
                             cos(2. * PI * **mSrcFreq * time +
                                 Math::phase(**mVoltageRef) + 2. / 3. * M_PI);
  }

  mEquivCurrent = **mIntfVoltage / **mResistance;
}

void EMT::Ph3::VoltageSourceNorton::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // Calculate v1 - v0
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

void EMT::Ph3::VoltageSourceNorton::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph3::VoltageSourceNorton::mnaCompPreStep(Real time,
                                                   Int timeStepCount) {
  updateState(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::VoltageSourceNorton::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::VoltageSourceNorton::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::VoltageSourceNorton::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  // signs are not verified
  (**mIntfCurrent)(0, 0) =
      mEquivCurrent(0, 0) - (**mIntfVoltage)(0, 0) / **mResistance;
  (**mIntfCurrent)(1, 0) =
      mEquivCurrent(1, 0) - (**mIntfVoltage)(1, 0) / **mResistance;
  (**mIntfCurrent)(2, 0) =
      mEquivCurrent(2, 0) - (**mIntfVoltage)(2, 0) / **mResistance;
}
