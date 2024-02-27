/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_Switch.h>

using namespace CPS;

DP::Ph1::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph1::Switch(mAttributes) {
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph1::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  return copy;
}

void DP::Ph1::Switch::initializeFromNodesAndTerminals(Real frequency) {

  Real impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;

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

void DP::Ph1::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                        Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  **mRightVector = Matrix::Zero(0, 0);
}

void DP::Ph1::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  Complex conductance = (**mIsClosed) ? Complex(1. / **mClosedResistance, 0)
                                      : Complex(1. / **mOpenResistance, 0);

  // Set diagonal entries
  if (terminalNotGrounded(0))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(0), conductance);
  if (terminalNotGrounded(1))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(1), conductance);
  // Set off diagonal entries
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(1), -conductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(0), -conductance);
  }

  SPDLOG_LOGGER_TRACE(mSLog, "-- Stamp ---");
  if (terminalNotGrounded(0))
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(conductance),
                        matrixNodeIndex(0), matrixNodeIndex(0));
  if (terminalNotGrounded(1))
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(conductance),
                        matrixNodeIndex(1), matrixNodeIndex(1));
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(-conductance),
                        matrixNodeIndex(0), matrixNodeIndex(1));
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(-conductance),
                        matrixNodeIndex(1), matrixNodeIndex(0));
  }
}

void DP::Ph1::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  Complex conductance = (closed) ? Complex(1. / **mClosedResistance, 0)
                                 : Complex(1. / **mOpenResistance, 0);

  // Set diagonal entries
  if (terminalNotGrounded(0))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(0), conductance);
  if (terminalNotGrounded(1))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(1), conductance);

  // Set off diagonal entries
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(1), -conductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(0), -conductance);
  }

  SPDLOG_LOGGER_TRACE(mSLog, "-- Stamp ---");
  if (terminalNotGrounded(0))
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(conductance),
                        matrixNodeIndex(0), matrixNodeIndex(0));
  if (terminalNotGrounded(1))
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(conductance),
                        matrixNodeIndex(1), matrixNodeIndex(1));
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(-conductance),
                        matrixNodeIndex(0), matrixNodeIndex(1));
    SPDLOG_LOGGER_TRACE(mSLog, "Add {:s} to system at ({:d},{:d})",
                        Logger::complexToString(-conductance),
                        matrixNodeIndex(1), matrixNodeIndex(0));
  }
}

void DP::Ph1::Switch::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {}

Bool DP::Ph1::Switch::mnaIsClosed() { return isClosed(); }

void DP::Ph1::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = (**mIsClosed)
                               ? (**mIntfVoltage)(0, 0) / **mClosedResistance
                               : (**mIntfVoltage)(0, 0) / **mOpenResistance;
}

void DP::Ph1::Switch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {

  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                      Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}
