/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph3_Inductor.h>

using namespace CPS;

SP::Ph3::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph3::Inductor(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr SP::Ph3::Inductor::clone(String name) {
  auto copy = Inductor::make(name, mLogLevel);
  copy->setParameters(**mInductance);
  return copy;
}

void SP::Ph3::Inductor::initializeFromNodesAndTerminals(Real frequency) {

  Real omega = 2 * PI * frequency;
  MatrixComp reactance = MatrixComp::Zero(3, 3);
  reactance << Complex(0, omega * (**mInductance)(0, 0)),
      Complex(0, omega * (**mInductance)(0, 1)),
      Complex(0, omega * (**mInductance)(0, 2)),
      Complex(0, omega * (**mInductance)(1, 0)),
      Complex(0, omega * (**mInductance)(1, 1)),
      Complex(0, omega * (**mInductance)(1, 2)),
      Complex(0, omega * (**mInductance)(2, 0)),
      Complex(0, omega * (**mInductance)(2, 1)),
      Complex(0, omega * (**mInductance)(2, 2));
  mSusceptance = reactance.inverse();

  // IntfVoltage initialization for each phase
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfVoltage)(1, 0) = (**mIntfVoltage)(0, 0) *
                           Complex(cos(-2. / 3. * M_PI), sin(-2. / 3. * M_PI));
  (**mIntfVoltage)(2, 0) = (**mIntfVoltage)(0, 0) *
                           Complex(cos(2. / 3. * M_PI), sin(2. / 3. * M_PI));
  **mIntfCurrent = mSusceptance * **mIntfVoltage;

  SPDLOG_LOGGER_INFO(mSLog, "--- Initialize according to power flow ---");
}

void SP::Ph3::Inductor::mnaCompInitialize(Real omega, Real timeStep,
                                          Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
}

void SP::Ph3::Inductor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mSusceptance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void SP::Ph3::Inductor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph3::Inductor::mnaCompPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void SP::Ph3::Inductor::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
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

void SP::Ph3::Inductor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mSusceptance * **mIntfVoltage;
}

// #### Tear Methods ####
void SP::Ph3::Inductor::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  // TODO
  Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx,
                           1. / mSusceptance(0, 0));
  Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx,
                           1. / mSusceptance(1, 0));
  Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx,
                           1. / mSusceptance(2, 0));
}
