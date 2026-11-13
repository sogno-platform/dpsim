/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>

using namespace CPS;

EMT::Ph3::Resistor::Resistor(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, false, true, logLevel),
      Base::Ph3::Resistor(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::Resistor::clone(String name) {
  auto copy = Resistor::make(name, mLogLevel);
  copy->setParameters(**mResistance);
  return copy;
}

void EMT::Ph3::Resistor::initializeFromNodesAndTerminals(Real frequency) {

  // IntfVoltage initialization for each phase
  MatrixComp vInitABC = Matrix::Zero(3, 1);
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
  **mIntfVoltage = vInitABC.real();

  // TODO: here, the custom implementation for matrix inversion is still used, because mResistance is still a generic matrix.
  // Change the definition in future, if valid. The author currently doesn't know if this is allowed (mResistance is defined in Base)
  // This is also done in the following
  Matrix mResistanceInv = Matrix::Zero(3, 3);
  Math::invertMatrix(**mResistance, mResistanceInv);
  **mIntfCurrent = (mResistanceInv * vInitABC).real();

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nResistance [Ohm]: {:s}"
                     "\nConductance [S]: {:s}",
                     Logger::matrixToString(**mResistance),
                     Logger::matrixToString(mResistanceInv));
  SPDLOG_LOGGER_INFO(
      mSLog,
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
}

void EMT::Ph3::Resistor::mnaCompInitialize(Real omega, Real timeStep,
                                           Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mConductance = (**mResistance).inverse();
  **mRightVector = Matrix::Zero(0, 0);
}

void EMT::Ph3::Resistor::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductanceMatrix(
      mConductance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);

  SPDLOG_LOGGER_INFO(mSLog, "\nConductance matrix: {:s}",
                     Logger::matrixToString(mConductance));
}

void EMT::Ph3::Resistor::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::Resistor::mnaCompPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  // Fuse the two related post-step updates. The current is calculated from
  // this step's voltage difference, so invoking the separate update methods
  // needlessly rereads the just-written voltage attribute and adds dispatch
  // overhead for every resistor in composite networks. Keep the standalone
  // update methods unchanged for callers that use them independently.
  Matrix &voltage = **mIntfVoltage;
  Matrix &current = **mIntfCurrent;
  voltage.setZero();
  if (terminalNotGrounded(1))
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase, 0) =
          Math::realFromVectorElement(**leftVector, matrixNodeIndex(1, phase));
  if (terminalNotGrounded(0))
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase, 0) -=
          Math::realFromVectorElement(**leftVector, matrixNodeIndex(0, phase));

  const Matrix &conductance = mConductance;
  for (UInt phase = 0; phase < 3; ++phase)
    current(phase, 0) = conductance(phase, 0) * voltage(0, 0) +
                        conductance(phase, 1) * voltage(1, 0) +
                        conductance(phase, 2) * voltage(2, 0);

  SPDLOG_LOGGER_DEBUG(mSLog, "\nVoltage: {:s}",
                      Logger::matrixToString(**mIntfVoltage));
  SPDLOG_LOGGER_DEBUG(mSLog, "\nCurrent: {:s}",
                      Logger::matrixToString(**mIntfCurrent));
  mSLog->flush();
}

void EMT::Ph3::Resistor::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  const bool terminal0Connected = terminalNotGrounded(0);
  const bool terminal1Connected = terminalNotGrounded(1);
  Matrix &voltage = **mIntfVoltage;
  for (UInt phase = 0; phase < 3; ++phase) {
    Real value = 0.;
    if (terminal1Connected)
      value = leftVector(matrixNodeIndex(1, phase), 0);
    if (terminal0Connected)
      value -= leftVector(matrixNodeIndex(0, phase), 0);
    voltage(phase, 0) = value;
  }
  SPDLOG_LOGGER_DEBUG(mSLog, "\nVoltage: {:s}",
                      Logger::matrixToString(**mIntfVoltage));
  mSLog->flush();
}

void EMT::Ph3::Resistor::mnaCompUpdateCurrent(const Matrix &leftVector) {
  // The dimensions are fixed at 3x3; scalar rows avoid dynamic GEMV setup in
  // this per-step post-processing path.
  const Matrix &voltage = **mIntfVoltage;
  Matrix &current = **mIntfCurrent;
  for (Int phase = 0; phase < 3; ++phase)
    current(phase, 0) = mConductance(phase, 0) * voltage(0, 0) +
                        mConductance(phase, 1) * voltage(1, 0) +
                        mConductance(phase, 2) * voltage(2, 0);
  SPDLOG_LOGGER_DEBUG(mSLog, "\nCurrent: {:s}",
                      Logger::matrixToString(**mIntfCurrent));
  mSLog->flush();
}

// #### Tear Methods ####
void EMT::Ph3::Resistor::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  MatrixFixedSizeComp<3, 3> conductance = Matrix::Zero(3, 3);
  conductance.real() = (**mResistance).inverse();
  // Set diagonal entries
  Math::addToMatrixElement(tearMatrix, mTearIdx * 3, mTearIdx * 3,
                           1. / conductance(0, 0).real());
  Math::addToMatrixElement(tearMatrix, mTearIdx * 3 + 1, mTearIdx * 3 + 1,
                           1. / conductance(1, 1).real());
  Math::addToMatrixElement(tearMatrix, mTearIdx * 3 + 2, mTearIdx * 3 + 2,
                           1. / conductance(2, 2).real());
}
