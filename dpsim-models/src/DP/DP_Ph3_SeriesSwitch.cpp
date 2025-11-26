/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_SeriesSwitch.h>

using namespace CPS;

DP::Ph3::SeriesSwitch::SeriesSwitch(String uid, String name,
                                    Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph1::Switch(mAttributes) {
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

void DP::Ph3::SeriesSwitch::initializeFromNodesAndTerminals(Real frequency) {

  mTerminals[0]->setPhaseType(PhaseType::ABC);
  mTerminals[1]->setPhaseType(PhaseType::ABC);

  Real impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  **mIntfVoltage = initialVoltage(1) - initialVoltage(0);
  **mIntfCurrent = **mIntfVoltage / impedance;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across phasor: \n{}"
                     "\nCurrent phasor: \n{}"
                     "\nTerminal 0 voltage phasor: \n{}"
                     "\nTerminal 1 voltage phasor: \n{}",
                     Logger::phasorToString(initialVoltage(0)(0, 0)),
                     Logger::phasorToString(initialVoltage(1)(0, 0)),
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)));
}

Bool DP::Ph3::SeriesSwitch::mnaIsClosed() { return **mIsClosed; }

void DP::Ph3::SeriesSwitch::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  **mRightVector = Matrix::Zero(0, 0);
}

void DP::Ph3::SeriesSwitch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  Complex conductance = (**mIsClosed) ? Complex(1. / **mClosedResistance, 0)
                                      : Complex(1. / **mOpenResistance, 0);

  MNAStampUtils::stampAdmittanceAs3x3ScalarMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::SeriesSwitch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  Complex conductance = (closed) ? Complex(1. / **mClosedResistance, 0)
                                 : Complex(1. / **mOpenResistance, 0);

  MNAStampUtils::stampAdmittanceAs3x3ScalarMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::SeriesSwitch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::SeriesSwitch::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::SeriesSwitch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }

  SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {} < {}",
                      std::abs((**mIntfVoltage)(0, 0)),
                      std::arg((**mIntfVoltage)(0, 0)));
}

void DP::Ph3::SeriesSwitch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  Real impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  **mIntfCurrent = **mIntfVoltage / impedance;

  SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}",
                      std::abs((**mIntfCurrent)(0, 0)),
                      std::arg((**mIntfCurrent)(0, 0)));
}

Bool DP::Ph3::SeriesSwitch::hasParameterChanged() {
  // Check if state of switch changed
  if (!(mIsClosedPrev == this->mnaIsClosed())) {
    mIsClosedPrev = this->mnaIsClosed();
    return 1; //recompute system matrix
  } else {
    return 0; // do not recompute system matrix
  }
};
