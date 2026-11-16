/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorDQTrapez.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::SynchronGeneratorDQTrapez::SynchronGeneratorDQTrapez(
    String uid, String name, Logger::Level logLevel)
    : SynchronGeneratorDQ(uid, name, logLevel) {

  mNumericalMethod = NumericalMethod::Trapezoidal;
}

EMT::Ph3::SynchronGeneratorDQTrapez::SynchronGeneratorDQTrapez(
    String name, Logger::Level logLevel)
    : SynchronGeneratorDQTrapez(name, name, logLevel) {}

void EMT::Ph3::SynchronGeneratorDQTrapez::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;

  SynchronGeneratorDQ::initializeMatrixAndStates();
  for (UInt row = 0; row < 3; ++row)
    for (UInt col = 0; col < 3; ++col)
      mVariableSystemMatrixEntries.emplace_back(matrixNodeIndex(0, row),
                                                matrixNodeIndex(0, col));
}

void EMT::Ph3::SynchronGeneratorDQTrapez::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfVoltage);
}

void EMT::Ph3::SynchronGeneratorDQTrapez::mnaCompPreStep(Real time,
                                                         Int timeStepCount) {
  stepInPerUnit(time); //former system solve (trapezoidal)
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::SynchronGeneratorDQTrapez::stepInPerUnit(Real time) {
  // Receives the voltage of time step k and has to
  // calculate the current output for time step k+1

  // Calculate per unit values and
  // transform per unit voltages from abc to dq0
  mVdq0 = abcToDq0Transform(mThetaMech, **mIntfVoltage) / mBase_V;
  mVsr(0, 0) = mVdq0(0, 0);
  mVsr(3, 0) = mVdq0(1, 0);
  mVsr(6, 0) = mVdq0(2, 0);
  // Current is already available in pu dq0 from previous step

  // Update of fd winding voltage from exciter
  if (mHasExciter) {
    Real Vpss = mHasPSS ? mPSS->step(**mOmMech, **mElecTorque, mVsr(0, 0),
                                     mVsr(3, 0), mTimeStep)
                        : 0.0;
    // Note: scaled by Rfd/Lmd to transform from exciter pu system
    // to the synchronous generator pu system
    mVsr(1, 0) =
        (mRfd / mLmd) * mExciter->step(mVsr(0, 0), mVsr(3, 0), mTimeStep, Vpss);
  }

  // Update of mechanical torque from turbine governor
  if (mHasGovernorAndTurbine) {
    Real Pgv = mGovernor->step(**mOmMech, mTimeStep);
    **mMechTorque = mTurbine->step(Pgv, mTimeStep);
  } else if (mHasTurbineGovernorType1)
    **mMechTorque = mTurbineGovernorType1->step(**mOmMech, mTimeStep);
  else if (mHasTurbineGovernor)
    **mMechTorque = mTurbineGovernor->step(
        **mOmMech, 1, mInitElecPower.real() / mNomPower, mTimeStep);

  // Calculation of electrical torque
  **mElecTorque = (mPsisr(3, 0) * mIsr(0, 0) - mPsisr(0, 0) * mIsr(3, 0));

  // Update mechanical rotor angle with respect to electrical angle
  // using Euler and previous states
  mThetaMech = mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);

  // Update of omega using Euler
  **mOmMech = **mOmMech + mTimeStep * (1. / (2. * **mInertia) *
                                       (**mMechTorque - **mElecTorque));

  // Update using euler
  **mDelta = **mDelta + mTimeStep * (**mOmMech - 1);

  // Integrate the machine states as a linear function of the newly solved
  // terminal voltage. The Norton matrix below carries that voltage
  // dependence into the network solve, avoiding an unstable explicit
  // current-source/line-capacitance feedback loop.
  mPreviousFluxes = mPsisr;
  const Matrix stateMatrix =
      mBase_OmElec * (mFluxStateSpaceMat + mOmegaFluxMat * **mOmMech);
  const Matrix identity = Matrix::Identity(mPsisr.rows(), mPsisr.rows());
  const Matrix lhs = identity - (mTimeStep / 2.0) * stateMatrix;
  const Matrix lhsInverse = lhs.inverse();
  mFluxF = lhsInverse * (identity + (mTimeStep / 2.0) * stateMatrix);

  Matrix inputSelector = Matrix::Zero(mPsisr.rows(), mPsisr.rows());
  inputSelector(0, 0) = 1.0;
  inputSelector(3, 3) = 1.0;
  inputSelector(6, 6) = 1.0;
  mFluxK = lhsInverse * (mTimeStep * mBase_OmElec) * inputSelector;
  mPsisr = mFluxF * mPreviousFluxes + mFluxK * mVsr;
  mIsr = mFluxToCurrentMat * mPsisr;

  Matrix currentSelector = Matrix::Zero(3, mIsr.rows());
  Matrix voltageSelector = Matrix::Zero(mVsr.rows(), 3);
  currentSelector(0, 0) = 1.0;
  currentSelector(1, 3) = 1.0;
  currentSelector(2, 6) = 1.0;
  voltageSelector(0, 0) = 1.0;
  voltageSelector(3, 1) = 1.0;
  voltageSelector(6, 2) = 1.0;
  const Matrix dqJacobian =
      currentSelector * mFluxToCurrentMat * mFluxK * voltageSelector;
  const Real theta = mThetaMech;
  Matrix dq0ToAbc(3, 3);
  Matrix abcToDq0(3, 3);
  dq0ToAbc << cos(theta), -sin(theta), 1., cos(theta - 2. * PI / 3.),
      -sin(theta - 2. * PI / 3.), 1., cos(theta + 2. * PI / 3.),
      -sin(theta + 2. * PI / 3.), 1.;
  abcToDq0 << 2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * PI / 3.),
      2. / 3. * cos(theta + 2. * PI / 3.), -2. / 3. * sin(theta),
      -2. / 3. * sin(theta - 2. * PI / 3.),
      -2. / 3. * sin(theta + 2. * PI / 3.), 1. / 3., 1. / 3., 1. / 3.;
  mNortonConductance = (mBase_I / mBase_V) * dq0ToAbc * dqJacobian * abcToDq0;
  mIdq0(0, 0) = mIsr(0, 0);
  mIdq0(1, 0) = mIsr(3, 0);
  mIdq0(2, 0) = mIsr(6, 0);
  **mIntfCurrent = mBase_I * dq0ToAbcTransform(mThetaMech, mIdq0);
}

void EMT::Ph3::SynchronGeneratorDQTrapez::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (!terminalNotGrounded(0))
    return;
  for (UInt row = 0; row < 3; ++row)
    for (UInt col = 0; col < 3; ++col)
      Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, row),
                               matrixNodeIndex(0, col),
                               mNortonConductance(row, col));
}

void EMT::Ph3::SynchronGeneratorDQTrapez::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  const Matrix compensationCurrent = mNortonConductance * **mIntfVoltage;
  for (UInt phase = 0; phase < 3; ++phase)
    Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                           -(**mIntfCurrent)(phase, 0) +
                               compensationCurrent(phase, 0));
}

void EMT::Ph3::SynchronGeneratorDQTrapez::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mVdq0 = abcToDq0Transform(mThetaMech, **mIntfVoltage) / mBase_V;
  mVsr(0, 0) = mVdq0(0, 0);
  mVsr(3, 0) = mVdq0(1, 0);
  mVsr(6, 0) = mVdq0(2, 0);

  mPsisr = mFluxF * mPreviousFluxes + mFluxK * mVsr;
  mIsr = mFluxToCurrentMat * mPsisr;
  mIdq0(0, 0) = mIsr(0, 0);
  mIdq0(1, 0) = mIsr(3, 0);
  mIdq0(2, 0) = mIsr(6, 0);
  **mIntfCurrent = mBase_I * dq0ToAbcTransform(mThetaMech, mIdq0);
}
