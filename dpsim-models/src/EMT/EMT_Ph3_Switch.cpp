/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_Switch.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, false, true, logLevel),
      Base::Ph3::Switch(mAttributes) {
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  return copy;
}

void EMT::Ph3::Switch::initializeFromNodesAndTerminals(Real frequency) {

  Matrix impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
  **mIntfVoltage = vInitABC.real();
  **mIntfCurrent = (impedance.inverse() * vInitABC).real();

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

void EMT::Ph3::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                         Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  **mRightVector = Matrix::Zero(0, 0);
}

Bool EMT::Ph3::Switch::mnaIsClosed() { return **mIsClosed; }

void EMT::Ph3::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MatrixFixedSize<3, 3> conductance = (**mIsClosed)
                                          ? (**mClosedResistance).inverse()
                                          : (**mOpenResistance).inverse();

  MNAStampUtils::stampConductanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);

  SPDLOG_LOGGER_TRACE(mSLog, "\nConductance matrix: {:s}",
                      Logger::matrixToString(conductance));
}

void EMT::Ph3::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  MatrixFixedSize<3, 3> conductance = (closed) ? (**mClosedResistance).inverse()
                                               : (**mOpenResistance).inverse();

  MNAStampUtils::stampConductanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);

  SPDLOG_LOGGER_TRACE(mSLog, "\nConductance matrix: {:s}",
                      Logger::matrixToString(conductance));
}

void EMT::Ph3::Switch::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {}

void EMT::Ph3::Switch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                       Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0
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

void EMT::Ph3::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  Matrix conductance = Matrix::Zero(3, 3);
  (**mIsClosed) ? Math::invertMatrix(**mClosedResistance, conductance)
                : Math::invertMatrix(**mOpenResistance, conductance);

  **mIntfCurrent = conductance * **mIntfVoltage;
}

Bool EMT::Ph3::Switch::hasParameterChanged() {
  // Check if state of switch changed
  if (!(mIsClosedPrev == this->mnaIsClosed())) {
    mIsClosedPrev = this->mnaIsClosed();
    return 1; //recompute system matrix
  } else {
    return 0; // do not recompute system matrix
  }
};
