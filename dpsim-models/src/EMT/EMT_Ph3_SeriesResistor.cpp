/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SeriesResistor.h>

using namespace CPS;

EMT::Ph3::SeriesResistor::SeriesResistor(String uid, String name,
                                         Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, false, true, logLevel),
      Base::Ph1::Resistor(mAttributes) {

  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SeriesResistor::clone(String name) {
  auto copy = SeriesResistor::make(name, mLogLevel);
  copy->setParameters(**mResistance);
  return copy;
}

void EMT::Ph3::SeriesResistor::initializeFromNodesAndTerminals(Real frequency) {

  // Convert line-line RMS PF voltage to balanced phase-ground peak values,
  // matching EMT::Ph3::Resistor and the time-domain signal convention.
  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
  **mIntfVoltage = vInitABC.real();
  **mIntfCurrent = (vInitABC / **mResistance).real();

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across amplitude and phase: \n{}"
                     "\nCurrent amplitude and phase: \n{}"
                     "\nTerminal 0 voltage amplitude and phase: \n{}"
                     "\nTerminal 1 voltage amplitude and phase: \n{}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorMatrixToString(**mIntfVoltage),
                     Logger::phasorMatrixToString(**mIntfCurrent),
                     Logger::phasorMatrixToString(initialVoltage(0)),
                     Logger::phasorMatrixToString(initialVoltage(1)));
}

void EMT::Ph3::SeriesResistor::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  **mRightVector = Matrix::Zero(0, 0);
}

void EMT::Ph3::SeriesResistor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  Real conductance = 1. / **mResistance;

  // Set diagonal entries
  if (terminalNotGrounded(0))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0),
                             matrixNodeIndices(0), conductance);
  if (terminalNotGrounded(1))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1),
                             matrixNodeIndices(1), conductance);
  // Set off diagonal entries
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0),
                             matrixNodeIndices(1), -conductance);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1),
                             matrixNodeIndices(0), -conductance);
  }

  if (terminalNotGrounded(0))
    SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance,
                       matrixNodeIndex(0, 0), matrixNodeIndex(0, 0));
  if (terminalNotGrounded(1))
    SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance,
                       matrixNodeIndex(1, 0), matrixNodeIndex(1, 0));
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", -conductance,
                       matrixNodeIndex(0, 0), matrixNodeIndex(1, 0));
    SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", -conductance,
                       matrixNodeIndex(1, 0), matrixNodeIndex(0, 0));
  }
}

void EMT::Ph3::SeriesResistor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {

  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::SeriesResistor::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::SeriesResistor::mnaCompUpdateVoltage(const Matrix &leftVector) {
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

  SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {}", (**mIntfVoltage)(0, 0));
}

void EMT::Ph3::SeriesResistor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = **mIntfVoltage / **mResistance;

  SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}", (**mIntfCurrent)(0, 0));
}
