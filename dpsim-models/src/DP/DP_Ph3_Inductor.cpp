/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_Inductor.h>

using namespace CPS;

DP::Ph3::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      Base::Ph3::Inductor(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);

  mEquivCurrent = MatrixComp::Zero(3, 1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::Inductor::clone(String name) {
  auto copy = Inductor::make(name, mLogLevel);
  copy->setParameters(**mInductance);
  return copy;
}

void DP::Ph3::Inductor::initializeFromNodesAndTerminals(Real frequency) {
  const Real omega = 2.0 * PI * frequency;

  MatrixComp reactance = MatrixComp::Zero(3, 3);

  for (UInt row = 0; row < 3; ++row) {
    for (UInt col = 0; col < 3; ++col) {
      reactance(row, col) = Complex(0.0, omega * (**mInductance)(row, col));
    }
  }

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);

  vInitABC(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));

  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;

  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  **mIntfVoltage = vInitABC;
  **mIntfCurrent = reactance.inverse() * vInitABC;

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

void DP::Ph3::Inductor::initVars(Real omega, Real timeStep) {
  mOmega = omega;

  Matrix a = timeStep / 2.0 * (**mInductance).inverse();

  const Real b = timeStep * omega / 2.0;

  Matrix equivCondReal = a / (1.0 + b * b);

  Matrix equivCondImag = -a * b / (1.0 + b * b);

  mEquivCond = MatrixComp::Zero(3, 3);

  for (UInt row = 0; row < 3; ++row) {
    for (UInt col = 0; col < 3; ++col) {
      mEquivCond(row, col) =
          Complex(equivCondReal(row, col), equivCondImag(row, col));
    }
  }

  const Real preCurrFracReal = (1.0 - b * b) / (1.0 + b * b);

  const Real preCurrFracImag = (-2.0 * b) / (1.0 + b * b);

  mPrevCurrFac = Complex(preCurrFracReal, preCurrFracImag);

  // Initialize the history source consistently with the initialized
  // phase-peak DP voltage/current envelopes.
  mEquivCurrent = mEquivCond * **mIntfVoltage + mPrevCurrFac * **mIntfCurrent;
}

void DP::Ph3::Inductor::mnaCompInitialize(Real omega, Real timeStep,
                                          Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  initVars(omega, timeStep);

  SPDLOG_LOGGER_INFO(mSLog, "Initial phase-A DP voltage envelope: {} < {} rad",
                     Math::abs((**mIntfVoltage)(0, 0)),
                     Math::phase((**mIntfVoltage)(0, 0)));

  SPDLOG_LOGGER_INFO(mSLog, "Initial phase-A DP current envelope: {} < {} rad",
                     Math::abs((**mIntfCurrent)(0, 0)),
                     Math::phase((**mIntfCurrent)(0, 0)));
}

void DP::Ph3::Inductor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mEquivCond, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Inductor::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  // Companion-model history source for the next time step.
  mEquivCurrent = mEquivCond * **mIntfVoltage + mPrevCurrFac * **mIntfCurrent;

  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                           mEquivCurrent(0, 0));

    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                           mEquivCurrent(1, 0));

    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                           mEquivCurrent(2, 0));
  }

  if (terminalNotGrounded(1)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0),
                           -mEquivCurrent(0, 0));

    Math::setVectorElement(rightVector, matrixNodeIndex(1, 1),
                           -mEquivCurrent(1, 0));

    Math::setVectorElement(rightVector, matrixNodeIndex(1, 2),
                           -mEquivCurrent(2, 0));
  }
}

void DP::Ph3::Inductor::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfVoltage);
  prevStepDependencies.push_back(mIntfCurrent);
}

void DP::Ph3::Inductor::mnaCompPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::Inductor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::Inductor::mnaCompPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::Inductor::mnaCompUpdateVoltage(const Matrix &leftVector) {
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
}

void DP::Ph3::Inductor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mEquivCond * **mIntfVoltage + mEquivCurrent;
}

// #### Tear Methods ####

void DP::Ph3::Inductor::mnaTearInitialize(Real omega, Real timeStep) {
  initVars(omega, timeStep);
}

void DP::Ph3::Inductor::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  Math::addToMatrixElement(tearMatrix, mTearIdx * 3, mTearIdx * 3,
                           1.0 / mEquivCond(0, 0));

  Math::addToMatrixElement(tearMatrix, mTearIdx * 3 + 1, mTearIdx * 3 + 1,
                           1.0 / mEquivCond(1, 1));

  Math::addToMatrixElement(tearMatrix, mTearIdx * 3 + 2, mTearIdx * 3 + 2,
                           1.0 / mEquivCond(2, 2));
}

void DP::Ph3::Inductor::mnaTearApplyVoltageStamp(Matrix &voltageVector) {
  mEquivCurrent = mEquivCond * **mIntfVoltage + mPrevCurrFac * **mIntfCurrent;

  Math::addToVectorElement(voltageVector, mTearIdx * 3,
                           mEquivCurrent(0, 0) / mEquivCond(0, 0));

  Math::addToVectorElement(voltageVector, mTearIdx * 3 + 1,
                           mEquivCurrent(1, 0) / mEquivCond(1, 1));

  Math::addToVectorElement(voltageVector, mTearIdx * 3 + 2,
                           mEquivCurrent(2, 0) / mEquivCond(2, 2));
}

void DP::Ph3::Inductor::mnaTearPostStep(MatrixComp voltage,
                                        MatrixComp current) {
  **mIntfVoltage = voltage;
  **mIntfCurrent = mEquivCond * voltage + mEquivCurrent;
}
