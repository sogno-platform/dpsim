/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_ResIndSeries.h>

using namespace CPS;

EMT::Ph3::ResIndSeries::ResIndSeries(String uid, String name,
                                     Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mInductance(mAttributes->create<Matrix>("L")),
      mResistance(mAttributes->create<Matrix>("R")) {

  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::ResIndSeries::clone(String name) {
  auto copy = ResIndSeries::make(name, mLogLevel);
  copy->setParameters(**mResistance, **mInductance);
  return copy;
}

void EMT::Ph3::ResIndSeries::setParameters(Matrix resistanceMatrix,
                                           Matrix inductanceMatrix) {
  **mResistance = resistanceMatrix;
  **mInductance = inductanceMatrix;

  //check initial value of inductance
  if ((**mInductance)(0, 0) == 0.0 || (**mInductance)(1, 1) == 0.0 ||
      (**mInductance)(2, 2) == 0.0) {
    std::string err = "Inductance of " + this->name() + " can not be zero!";
    throw std::invalid_argument(err);
  }
}

void EMT::Ph3::ResIndSeries::initializeFromNodesAndTerminals(Real frequency) {
  Real omega = 2. * PI * frequency;
  MatrixComp impedance = MatrixComp::Zero(3, 3);
  impedance << Complex((**mResistance)(0, 0), omega * (**mInductance)(0, 0)),
      0.0, 0.0, 0.0,
      Complex((**mResistance)(1, 1), omega * (**mInductance)(1, 1)), 0.0, 0.0,
      0.0, Complex((**mResistance)(2, 2), omega * (**mInductance)(2, 2));

  // IntfVoltage initialization for each phase
  MatrixComp vInitABC = Matrix::Zero(3, 1);
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
  **mIntfVoltage = vInitABC.real();
  **mIntfCurrent = (impedance.inverse() * vInitABC).real();

  mSLog->info("\nResistance matrix [Ohm]: {:s}"
              "\nInductance matrix [H]: {:s}"
              "\nImpedance  matrix [Ohm]: {:s}",
              Logger::matrixToString(**mResistance),
              Logger::matrixToString(**mInductance),
              Logger::matrixCompToString(impedance));
  mSLog->info(
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nTerminal 1 voltage: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)));
  mSLog->flush();
}

void EMT::Ph3::ResIndSeries::initVars(Real timeStep) {
  // Assumption: symmetric R and L matrix
  Real a = timeStep * (**mResistance)(0, 0) / (2. * (**mInductance)(0, 0));
  Real b = timeStep / (2. * (**mInductance)(0, 0));

  mEquivCond = b / (1. + a);
  mPrevCurrFac = (1. - a) / (1. + a);

  mEquivCurrent = mEquivCond * **mIntfVoltage + mPrevCurrFac * **mIntfCurrent;
}

void EMT::Ph3::ResIndSeries::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  initVars(timeStep);

  mSLog->info("\n--- MNA initialization ---"
              "\nInitial current {:s}"
              "\nEquiv. current {:s}"
              "\n--- MNA initialization finished ---",
              Logger::matrixToString(**mIntfCurrent),
              Logger::matrixToString(mEquivCurrent));
  mSLog->flush();
}

void EMT::Ph3::ResIndSeries::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {

  // Set diagonal entries
  if (terminalNotGrounded(0)) {
    // set upper left block, 3x3 entries
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(0, 0), mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1),
                             matrixNodeIndex(0, 1), mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2),
                             matrixNodeIndex(0, 2), mEquivCond);
  }
  if (terminalNotGrounded(1)) {
    // set buttom right block, 3x3 entries
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(1, 0), mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1),
                             matrixNodeIndex(1, 1), mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2),
                             matrixNodeIndex(1, 2), mEquivCond);
  }
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    // set buttom right block, 3x3 entries
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(1, 0), -mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1),
                             matrixNodeIndex(1, 1), -mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2),
                             matrixNodeIndex(1, 2), -mEquivCond);

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(0, 0), -mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1),
                             matrixNodeIndex(0, 1), -mEquivCond);
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2),
                             matrixNodeIndex(0, 2), -mEquivCond);
  }

  mSLog->info("\nEquivalent Conductance: {:}", mEquivCond);
}

void EMT::Ph3::ResIndSeries::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // Update internal state
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

void EMT::Ph3::ResIndSeries::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfVoltage);
  prevStepDependencies.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::ResIndSeries::mnaCompPreStep(Real time, Int timeStepCount) {
  this->mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::ResIndSeries::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::ResIndSeries::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::ResIndSeries::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) +=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) +=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) +=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}

void EMT::Ph3::ResIndSeries::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mEquivCond * **mIntfVoltage + mEquivCurrent;
}