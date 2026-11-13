/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_SynchronGeneratorDQ.h>

//Added for testing/printing purposes:
#include <fstream>
#include <iostream>

using namespace CPS;
using namespace std;

namespace {
const Complex kPositiveSequenceRotation(std::cos(2. / 3. * PI),
                                        std::sin(2. / 3. * PI));
const Complex kPositiveSequenceRotationSquared =
    std::pow(kPositiveSequenceRotation, 2);
} // namespace

DP::Ph3::SynchronGeneratorDQ::SynchronGeneratorDQ(String uid, String name,
                                                  Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      Base::SynchronGenerator(mAttributes) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

DP::Ph3::SynchronGeneratorDQ::SynchronGeneratorDQ(String name,
                                                  Logger::Level logLevel)
    : SynchronGeneratorDQ(name, name, logLevel) {}

DP::Ph3::SynchronGeneratorDQ::~SynchronGeneratorDQ() {}

void DP::Ph3::SynchronGeneratorDQ::initializeFromNodesAndTerminals(
    Real frequency) {
  if (mInitialValuesSet)
    return;

  // SystemTopology stores the generator's total three-phase PF power on the
  // phase-A terminal record, matching the machine power base.
  const Complex terminalPower = -terminal(0)->singlePower();
  const Complex terminalVoltage = initialSingleVoltage(0);
  const Real terminalVoltagePeak =
      RMS3PH_TO_PEAK1PH * Math::abs(terminalVoltage);
  Base::SynchronGenerator::setInitialValues(
      terminalPower.real(), terminalPower.imag(), terminalVoltagePeak,
      Math::phase(terminalVoltage), terminalPower.real());

  SPDLOG_LOGGER_INFO(
      mSLog,
      "Initialized DP Ph3 synchronous generator from power flow: "
      "P={} W, Q={} var, V_terminal_peak={} V, angle={} rad, f={} Hz",
      terminalPower.real(), terminalPower.imag(), terminalVoltagePeak,
      Math::phase(terminalVoltage), frequency);
}

void DP::Ph3::SynchronGeneratorDQ::setMachineParametersFundamentalPerUnit(
    Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
    Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd,
    Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia) {
  Base::SynchronGenerator::setBaseAndFundamentalPerUnitParameters(
      nomPower, nomVolt, nomFreq, nomFieldCur, poleNumber, Rs, Ll, Lmd, Lmq,
      Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
}

void DP::Ph3::SynchronGeneratorDQ::setParametersFundamentalPerUnit(
    Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
    Real Rs, Real Ll, Real Lmd, Real Lmq, Real Rfd, Real Llfd, Real Rkd,
    Real Llkd, Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2, Real inertia,
    Real initActivePower, Real initReactivePower, Real initTerminalVolt,
    Real initVoltAngle, Real initMechPower) {

  Base::SynchronGenerator::setBaseAndFundamentalPerUnitParameters(
      nomPower, nomVolt, nomFreq, nomFieldCur, poleNumber, Rs, Ll, Lmd, Lmq,
      Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
  Base::SynchronGenerator::setInitialValues(initActivePower, initReactivePower,
                                            initTerminalVolt, initVoltAngle,
                                            initMechPower);
}

void DP::Ph3::SynchronGeneratorDQ::initialize(Matrix frequencies) {
  SimPowerComp<Complex>::initialize(frequencies);
  mSystemOmega = frequencies(0, 0);
}

void DP::Ph3::SynchronGeneratorDQ::initializeMatrixAndStates() {
  // #### Compensation ####
  mCompensationOn = false;
  mCompensationCurrent = MatrixComp::Zero(3, 1);
  // Calculate real compensation resistance from per unit
  mRcomp = mRcomp * mBase_Z;

  // #### General setup ####
  // Create matrices for state space representation
  calcStateSpaceMatrixDQ();

  // steady state per unit initial value
  initPerUnitStates();

  mVdq0 = Matrix::Zero(3, 1);
  mIdq0 = Matrix::Zero(3, 1);
  if (mNumDampingWindings == 2) {
    mVdq0 << mVsr(0, 0), mVsr(3, 0), mVsr(6, 0);
    mIdq0 << mIsr(0, 0), mIsr(3, 0), mIsr(6, 0);
  } else {
    mVdq0 << mVsr(0, 0), mVsr(3, 0), mVsr(5, 0);
    mIdq0 << mIsr(0, 0), mIsr(3, 0), mIsr(5, 0);
  }

  **mIntfVoltage = mBase_V * dq0ToAbcTransform(mThetaMech, mVdq0);
  **mIntfCurrent = mBase_I * dq0ToAbcTransform(mThetaMech, mIdq0);
}

void DP::Ph3::SynchronGeneratorDQ::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (!mCompensationOn)
    return;

  Real conductance = 1. / mRcomp;

  if (terminalNotGrounded(0)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0)[0],
                             matrixNodeIndices(0)[0], Complex(conductance, 0));
    Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0)[1],
                             matrixNodeIndices(0)[1], Complex(conductance, 0));
    Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0)[2],
                             matrixNodeIndices(0)[2], Complex(conductance, 0));
    SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance,
                       matrixNodeIndices(0)[0], matrixNodeIndices(0)[0]);
    SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance,
                       matrixNodeIndices(0)[1], matrixNodeIndices(0)[1]);
    SPDLOG_LOGGER_INFO(mSLog, "Add {} to {}, {}", conductance,
                       matrixNodeIndices(0)[2], matrixNodeIndices(0)[2]);
  }
}

void DP::Ph3::SynchronGeneratorDQ::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (mCompensationOn)
    mCompensationCurrent = **mIntfVoltage / mRcomp;

  // If the interface current is positive, it is flowing out of the connected node and into ground.
  // Therefore, the generator is interfaced as a consumer but since the currents are reversed the equations
  // are in generator mode.
  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                           -(**mIntfCurrent)(0, 0) +
                               mCompensationCurrent(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                           -(**mIntfCurrent)(1, 0) +
                               mCompensationCurrent(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                           -(**mIntfCurrent)(2, 0) +
                               mCompensationCurrent(2, 0));
  }
}

void DP::Ph3::SynchronGeneratorDQ::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void DP::Ph3::SynchronGeneratorDQ::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph3::SynchronGeneratorDQ::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
}

Real DP::Ph3::SynchronGeneratorDQ::electricalTorque() const {
  return **mElecTorque * mBase_T;
}

Real DP::Ph3::SynchronGeneratorDQ::rotationalSpeed() const {
  return **mOmMech * mBase_OmMech;
}

Real DP::Ph3::SynchronGeneratorDQ::rotorPosition() const { return mThetaMech; }

MatrixFixedSize<3, 1>
DP::Ph3::SynchronGeneratorDQ::abcToDq0Transform(Real theta,
                                                MatrixComp &abcVector) {
  // Balanced positive-sequence projection, followed by e^(-j theta). Keep
  // the sequence rotation constants shared instead of rebuilding 3x3
  // transform matrices and powers of alpha at every generator timestep.
  const Complex positiveSequence =
      (abcVector(0, 0) + kPositiveSequenceRotation * abcVector(1, 0) +
       kPositiveSequenceRotationSquared * abcVector(2, 0)) /
      3.;
  const Complex thetaCompInv(std::cos(theta), -std::sin(theta));
  const Complex pnz = positiveSequence * thetaCompInv;

  MatrixFixedSize<3, 1> dq0Vector;
  dq0Vector << pnz.real(), pnz.imag(), 0.;

  return dq0Vector;
}

MatrixFixedSizeComp<3, 1> DP::Ph3::SynchronGeneratorDQ::dq0ToAbcTransform(
    Real theta, const Eigen::Ref<const Matrix> &dq0) {
  const Complex thetaComp(std::cos(theta), std::sin(theta));
  const Complex positiveSequence = Complex(dq0(0, 0), dq0(1, 0)) * thetaComp;
  MatrixFixedSizeComp<3, 1> abcCompVector;
  abcCompVector << positiveSequence,
      kPositiveSequenceRotationSquared * positiveSequence,
      kPositiveSequenceRotation * positiveSequence;

  return abcCompVector;
}

void DP::Ph3::SynchronGeneratorDQ::trapezoidalFluxStates() {}

void DP::Ph3::SynchronGeneratorDQ::trapezoidalCurrentStates() {}
