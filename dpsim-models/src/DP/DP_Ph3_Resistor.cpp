/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_Resistor.h>

using namespace CPS;

DP::Ph3::Resistor::Resistor(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph3::Resistor(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);

  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::Resistor::clone(String name) {
  auto copy = Resistor::make(name, mLogLevel);
  copy->setParameters(**mResistance);
  return copy;
}

void DP::Ph3::Resistor::initializeFromNodesAndTerminals(Real frequency) {
  // ---------------------------------------------------------------------------
  // DP / PF CONVENTION CONVERSION
  // ---------------------------------------------------------------------------
  // Power-flow node voltages are line-line RMS phasors.
  // DP::Ph3 states use phase-peak complex envelopes.
  //
  // The previous implementation constructed B/C from mIntfVoltage(A) without
  // first assigning phase A from the PF terminal voltages. This made standalone
  // resistor initialization dependent on stale/default interface state.
  // ---------------------------------------------------------------------------

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);

  vInitABC(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));

  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;

  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  **mIntfVoltage = vInitABC;

  **mIntfCurrent = (**mResistance).inverse() * **mIntfVoltage;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nDP convention: phase-peak complex envelopes"
      "\nVoltage across:"
      "\n{:s}"
      "\nCurrent:"
      "\n{:s}"
      "\nTerminal 0 PF voltage converted to phase peak: {:s}"
      "\nTerminal 1 PF voltage converted to phase peak: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::matrixCompToString(**mIntfVoltage),
      Logger::matrixCompToString(**mIntfCurrent),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)));
}

void DP::Ph3::Resistor::mnaCompInitialize(Real omega, Real timeStep,
                                          Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  **mRightVector = Matrix::Zero(0, 0);
}

void DP::Ph3::Resistor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MatrixFixedSizeComp<3, 3> conductance = MatrixFixedSizeComp<3, 3>::Zero();

  conductance.real() = (**mResistance).inverse();

  MNAStampUtils::stampAdmittanceMatrix(
      conductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Resistor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::Resistor::mnaCompPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::Resistor::mnaCompUpdateVoltage(const Matrix &leftVector) {
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
    (**mIntfVoltage)(0, 0) -=
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

    (**mIntfVoltage)(1, 0) -=
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 1));

    (**mIntfVoltage)(2, 0) -=
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }

  SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {} < {}",
                      std::abs((**mIntfVoltage)(0, 0)),
                      std::arg((**mIntfVoltage)(0, 0)));
}

void DP::Ph3::Resistor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = (**mResistance).inverse() * **mIntfVoltage;

  SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}",
                      std::abs((**mIntfCurrent)(0, 0)),
                      std::arg((**mIntfCurrent)(0, 0)));
}

// #### Tear Methods ####

void DP::Ph3::Resistor::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  MatrixFixedSizeComp<3, 3> conductance = MatrixFixedSizeComp<3, 3>::Zero();

  conductance.real() = (**mResistance).inverse();

  Math::addToMatrixElement(tearMatrix, mTearIdx * 3, mTearIdx * 3,
                           1.0 / conductance(0, 0).real());

  Math::addToMatrixElement(tearMatrix, mTearIdx * 3 + 1, mTearIdx * 3 + 1,
                           1.0 / conductance(1, 1).real());

  Math::addToMatrixElement(tearMatrix, mTearIdx * 3 + 2, mTearIdx * 3 + 2,
                           1.0 / conductance(2, 2).real());
}
