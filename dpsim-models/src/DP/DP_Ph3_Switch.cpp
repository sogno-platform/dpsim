// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_Switch.h>

using namespace CPS;

DP::Ph3::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph3::Switch(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  return copy;
}

void DP::Ph3::Switch::initializeFromNodesAndTerminals(Real frequency) {
  mTerminals[0]->setPhaseType(PhaseType::ABC);
  mTerminals[1]->setPhaseType(PhaseType::ABC);

  Matrix impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  // Voltage across component is defined as V1 - V0.
  **mIntfVoltage = initialVoltage(1) - initialVoltage(0);
  **mIntfCurrent = impedance.inverse() * **mIntfVoltage;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across phasor: \n{:s}"
                     "\nCurrent phasor: \n{:s}"
                     "\nTerminal 0 voltage phasor: \n{:s}"
                     "\nTerminal 1 voltage phasor: \n{:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorMatrixToString(**mIntfVoltage),
                     Logger::phasorMatrixToString(**mIntfCurrent),
                     Logger::phasorMatrixToString(initialVoltage(0)),
                     Logger::phasorMatrixToString(initialVoltage(1)));
}

void DP::Ph3::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                        Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  **mRightVector = Matrix::Zero(0, 0);
}

Bool DP::Ph3::Switch::mnaIsClosed() { return **mIsClosed; }

void DP::Ph3::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MatrixFixedSizeComp<3, 3> conductance = Matrix::Zero(3, 3);
  conductance.real() = (**mIsClosed) ? (**mClosedResistance).inverse()
                                     : (**mOpenResistance).inverse();

  MNAStampUtils::stampAdmittanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  MatrixFixedSizeComp<3, 3> conductance = Matrix::Zero(3, 3);
  conductance.real() = (closed) ? (**mClosedResistance).inverse()
                                : (**mOpenResistance).inverse();

  MNAStampUtils::stampAdmittanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Switch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                      Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0.
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
}

void DP::Ph3::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  MatrixFixedSizeComp<3, 3> conductance = Matrix::Zero(3, 3);
  conductance.real() = (**mIsClosed) ? (**mClosedResistance).inverse()
                                     : (**mOpenResistance).inverse();

  **mIntfCurrent = conductance * **mIntfVoltage;
}

Bool DP::Ph3::Switch::hasParameterChanged() {
  // Recompute the system matrix only when the switch state changed.
  if (!(mIsClosedPrev == this->mnaIsClosed())) {
    mIsClosedPrev = this->mnaIsClosed();
    return 1;
  } else {
    return 0;
  }
}
