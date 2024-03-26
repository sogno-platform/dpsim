/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_VoltageSourceNorton.h>

using namespace CPS;

DP::Ph1::VoltageSourceNorton::VoltageSourceNorton(String uid, String name,
                                                  Logger::Level logLevel)
    : Base::Ph1::VoltageSource(mAttributes), MNASimPowerComp<Complex>(
                                                 uid, name, true, true,
                                                 logLevel),
      mResistance(mAttributes->create<Real>("R")) {
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph1::VoltageSourceNorton::clone(String name) {
  auto copy = VoltageSourceNorton::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef, **mSrcFreq, **mResistance);
  return copy;
}

void DP::Ph1::VoltageSourceNorton::setParameters(Complex voltage, Real srcFreq,
                                                 Real resistance) {
  Base::Ph1::VoltageSource::setParameters(voltage, srcFreq);

  **mResistance = resistance;
  mConductance = 1. / **mResistance;
  mEquivCurrent = **mVoltageRef / **mResistance;

  mParametersSet = true;
}

void DP::Ph1::VoltageSourceNorton::setVoltageRef(Complex voltage) const {
  **mVoltageRef = voltage;
}

void DP::Ph1::VoltageSourceNorton::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

  (**mIntfVoltage)(0, 0) = **mVoltageRef;
}

void DP::Ph1::VoltageSourceNorton::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Apply matrix stamp for equivalent resistance
  if (terminalNotGrounded(0))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(0), Complex(mConductance, 0));
  if (terminalNotGrounded(1))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(1), Complex(mConductance, 0));
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(1), Complex(-mConductance, 0));
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(0), Complex(-mConductance, 0));
  }

  if (terminalNotGrounded(0))
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(mConductance, 0)),
                       matrixNodeIndex(0), matrixNodeIndex(0));
  if (terminalNotGrounded(1))
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(mConductance, 0)),
                       matrixNodeIndex(1), matrixNodeIndex(1));
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-mConductance, 0)),
                       matrixNodeIndex(0), matrixNodeIndex(1));
    SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
                       Logger::complexToString(Complex(-mConductance, 0)),
                       matrixNodeIndex(1), matrixNodeIndex(0));
  }
}

void DP::Ph1::VoltageSourceNorton::initializeFromNodesAndTerminals(
    Real frequency) {}

void DP::Ph1::VoltageSourceNorton::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  mEquivCurrent = (**mIntfVoltage)(0, 0) / **mResistance;

  // Apply matrix stamp for equivalent current source
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0), -mEquivCurrent);
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1), mEquivCurrent);
}

void DP::Ph1::VoltageSourceNorton::updateState(Real time) {
  if (**mSrcFreq >= 0) {
    (**mIntfVoltage)(0, 0) = Complex(
        Math::abs(**mVoltageRef) *
            cos(time * 2. * PI * **mSrcFreq + Math::phase(**mVoltageRef)),
        Math::abs(**mVoltageRef) *
            sin(time * 2. * PI * **mSrcFreq + Math::phase(**mVoltageRef)));
  } else {
    // If source frequency -1, use system frequency.
    (**mIntfVoltage)(0, 0) = **mVoltageRef;
  }
}
void DP::Ph1::VoltageSourceNorton::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::VoltageSourceNorton::mnaCompPreStep(Real time,
                                                  Int timeStepCount) {
  updateState(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::VoltageSourceNorton::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph1::VoltageSourceNorton::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph1::VoltageSourceNorton::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // Calculate v1 - v0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::VoltageSourceNorton::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  // TODO: verify signs
  (**mIntfCurrent)(0, 0) =
      mEquivCurrent - (**mIntfVoltage)(0, 0) / **mResistance;
}
