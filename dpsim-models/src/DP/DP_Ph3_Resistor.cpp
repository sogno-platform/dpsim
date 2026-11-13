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
  mConductance.real() = (**mResistance).inverse();
  **mRightVector = Matrix::Zero(0, 0);
}

void DP::Ph3::Resistor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mConductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
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
  const bool terminal0Connected = terminalNotGrounded(0);
  const bool terminal1Connected = terminalNotGrounded(1);
  const Matrix::Index imaginaryOffset = leftVector.rows() / 2;
  for (UInt phase = 0; phase < 3; ++phase) {
    Complex voltage(0., 0.);
    if (terminal1Connected) {
      const Matrix::Index row = matrixNodeIndex(1, phase);
      voltage =
          Complex(leftVector(row, 0), leftVector(row + imaginaryOffset, 0));
    }
    if (terminal0Connected) {
      const Matrix::Index row = matrixNodeIndex(0, phase);
      voltage -=
          Complex(leftVector(row, 0), leftVector(row + imaginaryOffset, 0));
    }
    (**mIntfVoltage)(phase, 0) = voltage;
  }

  SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {} < {}",
                      std::abs((**mIntfVoltage)(0, 0)),
                      std::arg((**mIntfVoltage)(0, 0)));
}

void DP::Ph3::Resistor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  // The dimensions are fixed at 3x3; scalar rows avoid dynamic GEMV setup in
  // this per-step post-processing path.
  const MatrixComp &voltage = **mIntfVoltage;
  MatrixComp &current = **mIntfCurrent;
  for (Int phase = 0; phase < 3; ++phase)
    current(phase, 0) = mConductance(phase, 0) * voltage(0, 0) +
                        mConductance(phase, 1) * voltage(1, 0) +
                        mConductance(phase, 2) * voltage(2, 0);

  SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}",
                      std::abs((**mIntfCurrent)(0, 0)),
                      std::arg((**mIntfCurrent)(0, 0)));
}

// #### Tear Methods ####

void DP::Ph3::Resistor::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  // Keep mutual phase coupling in the tear system. Stamping only the
  // diagonal here disagrees with the full resistance matrix used by MNA.
  for (UInt row = 0; row < 3; ++row) {
    for (UInt col = 0; col < 3; ++col) {
      Math::addToMatrixElement(tearMatrix, mTearIdx * 3 + row,
                               mTearIdx * 3 + col,
                               Complex((**mResistance)(row, col), 0.0));
    }
  }
}
