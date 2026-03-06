/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include <dpsim-models/DP/DP_Ph1_ControlledVoltageSource.h>

using namespace CPS;

DP::Ph1::ControlledVoltageSource::ControlledVoltageSource(
    String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mVoltageRef(mAttributes->createDynamic<Complex>(
          "V_ref")) // rms-value, phase-to-phase
{
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

void DP::Ph1::ControlledVoltageSource::setParameters(Complex voltageRef) {
  **mVoltageRef = voltageRef;

  mParametersSet = true;
}

void DP::Ph1::ControlledVoltageSource::initializeFromNodesAndTerminals(
    Real frequency) {
  SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from node voltages ---");
  // TODO: this approach currently overwrites voltage reference set from outside, when not using setParameters
  if (!mParametersSet) {
    // initialize voltage reference as zeroes
    SPDLOG_LOGGER_INFO(mSLog,
                       "\nReference voltage: {:s}"
                       "\nTerminal 0 voltage: {:s}"
                       "\nTerminal 1 voltage: {:s}",
                       Logger::phasorToString(**mVoltageRef),
                       Logger::phasorToString(initialSingleVoltage(0)),
                       Logger::phasorToString(initialSingleVoltage(1)));
  } else {
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\nInitialization from node voltages omitted (parameter already set)."
        "\nReference voltage: {:s}",
        Logger::phasorToString(**mVoltageRef));
  }
  SPDLOG_LOGGER_INFO(mSLog, "\n--- Initialization from node voltages ---");
  mSLog->flush();
}

SimPowerComp<Complex>::Ptr DP::Ph1::ControlledVoltageSource::clone(String name) {
  auto copy = ControlledVoltageSource::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef);
  return copy;
}

void DP::Ph1::ControlledVoltageSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

    (**mIntfVoltage)(0, 0) = **mVoltageRef;

    SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- MNA initialization ---"
                     "\nInitial voltage {:s}"
                     "\nInitial current {:s}"
                     "\n--- MNA initialization finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)));
}

void DP::Ph1::ControlledVoltageSource::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    if (terminalNotGrounded(0)) {
      Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                             matrixNodeIndex(0), Complex(-1, 0), mNumFreqs,
                             freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(0),
                             mVirtualNodes[0]->matrixNodeIndex(),
                             Complex(-1, 0), mNumFreqs, freq);
    }
    if (terminalNotGrounded(1)) {
      Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                             matrixNodeIndex(1), Complex(1, 0), mNumFreqs,
                             freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(1),
                             mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0),
                             mNumFreqs, freq);
    }

    SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freq);
    if (terminalNotGrounded(0)) {
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1.,
                         matrixNodeIndex(0),
                         mVirtualNodes[0]->matrixNodeIndex());
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1.,
                         mVirtualNodes[0]->matrixNodeIndex(),
                         matrixNodeIndex(0));
    }
    if (terminalNotGrounded(1)) {
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1.,
                         mVirtualNodes[0]->matrixNodeIndex(),
                         matrixNodeIndex(1));
      SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1.,
                         matrixNodeIndex(1),
                         mVirtualNodes[0]->matrixNodeIndex());
    }
  }
}

void DP::Ph1::ControlledVoltageSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // TODO: Is this correct with two nodes not gnd?
  Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(),
                         (**mIntfVoltage)(0, 0), mNumFreqs);
  SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
                      Logger::complexToString((**mIntfVoltage)(0, 0)),
                      mVirtualNodes[0]->matrixNodeIndex());
}

void DP::Ph1::ControlledVoltageSource::updateVoltage(Real time) {
  (**mIntfVoltage)(0, 0) = **mVoltageRef;

  SPDLOG_LOGGER_DEBUG(mSLog, "\nUpdate Voltage: {:s}",
                      Logger::matrixToString(**mIntfVoltage));
}

void DP::Ph1::ControlledVoltageSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph1::ControlledVoltageSource::mnaCompPreStep(Real time,
                                                       Int timeStepCount) {
  updateVoltage(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::ControlledVoltageSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
};

void DP::Ph1::ControlledVoltageSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph1::ControlledVoltageSource::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    (**mIntfCurrent)(0, freq) = Math::complexFromVectorElement(
        leftVector, mVirtualNodes[0]->matrixNodeIndex(), mNumFreqs, freq);
  }
}
