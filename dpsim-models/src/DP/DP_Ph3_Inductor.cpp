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
  MatrixComp susceptance = reactance.inverse();
  // IntfVoltage initialization for each phase
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  Real voltMag = Math::abs((**mIntfVoltage)(0, 0));
  Real voltPhase = Math::phase((**mIntfVoltage)(0, 0));
  (**mIntfVoltage)(1, 0) = Complex(voltMag * cos(voltPhase - 2. / 3. * M_PI),
                                   voltMag * sin(voltPhase - 2. / 3. * M_PI));
  (**mIntfVoltage)(2, 0) = Complex(voltMag * cos(voltPhase + 2. / 3. * M_PI),
                                   voltMag * sin(voltPhase + 2. / 3. * M_PI));

  **mIntfCurrent = susceptance * **mIntfVoltage;

  //TODO
  SPDLOG_LOGGER_INFO(mSLog, "--- Initialize according to power flow ---");
}

void DP::Ph3::Inductor::initVars(Real omega, Real timeStep) {
  Matrix a = timeStep / 2. * (**mInductance).inverse();
  Real b = timeStep * omega / 2.;

  Matrix equivCondReal = a / (1. + b * b);
  Matrix equivCondImag = -a * b / (Real(1.) + b * b);
  mEquivCond = MatrixComp::Zero(3, 3);
  mEquivCond << Complex(equivCondReal(0, 0), equivCondImag(0, 0)),
      Complex(equivCondReal(0, 1), equivCondImag(0, 1)),
      Complex(equivCondReal(0, 2), equivCondImag(0, 2)),
      Complex(equivCondReal(1, 0), equivCondImag(1, 0)),
      Complex(equivCondReal(1, 1), equivCondImag(1, 1)),
      Complex(equivCondReal(1, 2), equivCondImag(1, 2)),
      Complex(equivCondReal(2, 0), equivCondImag(2, 0)),
      Complex(equivCondReal(2, 1), equivCondImag(2, 1)),
      Complex(equivCondReal(2, 2), equivCondImag(2, 2));

  Real preCurrFracReal = (1. - b * b) / (1. + b * b);
  Real preCurrFracImag = (-2. * b) / (1. + b * b);
  mPrevCurrFac = Complex(preCurrFracReal, preCurrFracImag);

  // TODO: check if this is correct or if it should be only computed before the step
  mEquivCurrent = mEquivCond * **mIntfVoltage + mPrevCurrFac * **mIntfCurrent;
}

void DP::Ph3::Inductor::mnaCompInitialize(Real omega, Real timeStep,
                                          Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  initVars(omega, timeStep);

  SPDLOG_LOGGER_INFO(mSLog, "Initial voltage {}",
                     Math::abs((**mIntfVoltage)(0, 0)));
}

void DP::Ph3::Inductor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mEquivCond, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::Inductor::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {

  // Calculate equivalent current source for next time step
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
  // actually depends on L, but then we'd have to modify the system matrix anyway
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

void DP::Ph3::Inductor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mEquivCond * **mIntfVoltage + mEquivCurrent;
}

void DP::Ph3::Inductor::mnaTearInitialize(Real omega, Real timeStep) {
  initVars(omega, timeStep);
}

void DP::Ph3::Inductor::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {}

void DP::Ph3::Inductor::mnaTearApplyVoltageStamp(Matrix &voltageVector) {}

void DP::Ph3::Inductor::mnaTearPostStep(Complex voltage, Complex current) {}
