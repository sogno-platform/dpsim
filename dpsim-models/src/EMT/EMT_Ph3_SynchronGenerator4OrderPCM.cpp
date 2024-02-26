/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator4OrderPCM.h>

using namespace CPS;

EMT::Ph3::SynchronGenerator4OrderPCM::SynchronGenerator4OrderPCM(
    const String &uid, const String &name, Logger::Level logLevel)
    : Base::ReducedOrderSynchronGenerator<Real>(uid, name, logLevel),
      mEdq0_t(mAttributes->create<Matrix>("Edq0_t")) {

  mPhaseType = PhaseType::ABC;
  setTerminalNumber(1);

  // Initialize attributes
  mNumIter = mAttributes->create<Int>("NIterations", 0);

  // model variables
  **mEdq0_t = Matrix::Zero(3, 1);
}

EMT::Ph3::SynchronGenerator4OrderPCM::SynchronGenerator4OrderPCM(
    const String &name, Logger::Level logLevel)
    : SynchronGenerator4OrderPCM(name, name, logLevel) {}

void EMT::Ph3::SynchronGenerator4OrderPCM::specificInitialization() {

  // calculate state representation matrix
  calculateStateSpaceMatrices();

  // initial voltage behind the transient reactance in the dq0 reference frame
  (**mEdq0_t)(0, 0) = (**mVdq0)(0, 0) - (**mIdq0)(1, 0) * mLq_t;
  (**mEdq0_t)(1, 0) = (**mVdq0)(1, 0) + (**mIdq0)(0, 0) * mLd_t;
  (**mEdq0_t)(2, 0) = 0.0;

  SPDLOG_LOGGER_DEBUG(mSLog,
                      "\n--- Model specific initialization  ---"
                      "\nInitial Ed_t (per unit): {:f}"
                      "\nInitial Eq_t (per unit): {:f}"
                      "\nMax number of iterations: {:d}"
                      "\nTolerance: {:f}"
                      "\nSG Model: 4 Order PCM"
                      "\n--- Model specific initialization finished ---",

                      (**mEdq0_t)(0, 0), (**mEdq0_t)(1, 0), mMaxIter,
                      mTolerance);
  mSLog->flush();
}

void EMT::Ph3::SynchronGenerator4OrderPCM::calculateStateSpaceMatrices() {
  // Initialize matrices of state representation
  mAStateSpace << -mLq / mTq0_t / mLq_t, 0.0, 0.0, 0, -mLd / mTd0_t / mLd_t,
      0.0, 0, 0.0, 0.0;
  mBStateSpace << (mLq - mLq_t) / mTq0_t / mLq_t, 0.0, 0.0, 0.0,
      (mLd - mLd_t) / mTd0_t / mLd_t, 0.0, 0.0, 0.0, 0.0;
  mCStateSpace << 0.0, 1. / mTd0_t, 0.0;
  // Precalculate trapezoidal based matrices (avoids redundant matrix inversions in correction steps)
  Math::calculateStateSpaceTrapezoidalMatrices(
      mAStateSpace, mBStateSpace, mCStateSpace, mTimeStep, mAdTrapezoidal,
      mBdTrapezoidal, mCdTrapezoidal);
}

void EMT::Ph3::SynchronGenerator4OrderPCM::stepInPerUnit() {
  // set number of iterations equal to zero
  **mNumIter = 0;

  // Predictor step (euler)

  // store values currently at t=k for later use
  mEdq0tPrevStep = **mEdq0_t;
  mIdq0PrevStep = **mIdq0;
  mVdq0PrevStep = **mVdq0;

  // predict emf at t=k+1 (euler) using
  (**mEdq0_t) = Math::StateSpaceEuler(**mEdq0_t, mAStateSpace, mBStateSpace,
                                      mCStateSpace * **mEf, mTimeStep, **mVdq0);

  // predict stator currents at t=k+1 (assuming Vdq0(k+1)=Vdq0(k))
  (**mIdq0)(0, 0) = ((**mEdq0_t)(1, 0) - (**mVdq0)(1, 0)) / mLd_t;
  (**mIdq0)(1, 0) = ((**mVdq0)(0, 0) - (**mEdq0_t)(0, 0)) / mLq_t;
  (**mIdq0)(2, 0) = 0.0;

  // convert currents into the abc domain
  **mIntfCurrent = inverseParkTransform(**mThetaMech, **mIdq0);
  **mIntfCurrent = **mIntfCurrent * mBase_I;
}

void EMT::Ph3::SynchronGenerator4OrderPCM::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                         (**mIntfCurrent)(0, 0));
  Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                         (**mIntfCurrent)(1, 0));
  Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                         (**mIntfCurrent)(2, 0));
}

void EMT::Ph3::SynchronGenerator4OrderPCM::correctorStep() {
  // increase number of iterations
  **mNumIter = **mNumIter + 1;

  // correct electrical vars
  // calculate emf at j and k+1 (trapezoidal rule)
  (**mEdq0_t) = Math::applyStateSpaceTrapezoidalMatrices(
      mAdTrapezoidal, mBdTrapezoidal, mCdTrapezoidal * **mEf, mEdq0tPrevStep,
      **mVdq0, mVdq0PrevStep);

  // calculate corrected stator currents at t=k+1 (assuming Vdq(k+1)=VdqPrevIter(k+1))
  (**mIdq0)(0, 0) = ((**mEdq0_t)(1, 0) - (**mVdq0)(1, 0)) / mLd_t;
  (**mIdq0)(1, 0) = ((**mVdq0)(0, 0) - (**mEdq0_t)(0, 0)) / mLq_t;

  // convert currents into the abc domain
  **mIntfCurrent = inverseParkTransform(**mThetaMech, **mIdq0);
  **mIntfCurrent = **mIntfCurrent * mBase_I;

  // stamp currents
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::SynchronGenerator4OrderPCM::updateVoltage(
    const Matrix &leftVector) {
  // store voltage value currently at j-1 for later use
  mVdq0PrevIter = **mVdq0;

  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));

  // convert Vdq into abc domain
  **mVdq0 = parkTransform(**mThetaMech, **mIntfVoltage);
  **mVdq0 = **mVdq0 / mBase_V;
}

bool EMT::Ph3::SynchronGenerator4OrderPCM::requiresIteration() {
  if (**mNumIter >= mMaxIter) {
    // maximum number of iterations reached
    return false;
  } else if (**mNumIter == 0) {
    // no corrector step has been performed yet,
    // convergence cannot be confirmed
    return true;
  } else {
    // check voltage convergence according to tolerance
    Matrix voltageDifference = **mVdq0 - mVdq0PrevIter;
    if (Math::abs(voltageDifference(0, 0)) > mTolerance ||
        Math::abs(voltageDifference(1, 0)) > mTolerance)
      return true;
    else
      return false;
  }
}

void EMT::Ph3::SynchronGenerator4OrderPCM::mnaCompPostStep(
    const Matrix &leftVector) {}

Matrix
EMT::Ph3::SynchronGenerator4OrderPCM::parkTransform(Real theta,
                                                    const Matrix &abcVector) {
  Matrix dq0Vector(3, 1);
  Matrix abcToDq0(3, 3);

  // Park transform according to Kundur
  abcToDq0 << 2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * PI / 3.),
      2. / 3. * cos(theta + 2. * PI / 3.), -2. / 3. * sin(theta),
      -2. / 3. * sin(theta - 2. * PI / 3.),
      -2. / 3. * sin(theta + 2. * PI / 3.), 1. / 3., 1. / 3., 1. / 3.;

  dq0Vector = abcToDq0 * abcVector;

  return dq0Vector;
}

Matrix EMT::Ph3::SynchronGenerator4OrderPCM::inverseParkTransform(
    Real theta, const Matrix &dq0Vector) {
  Matrix abcVector(3, 1);
  Matrix dq0ToAbc(3, 3);

  // Park transform according to Kundur
  dq0ToAbc << cos(theta), -sin(theta), 1., cos(theta - 2. * PI / 3.),
      -sin(theta - 2. * PI / 3.), 1., cos(theta + 2. * PI / 3.),
      -sin(theta + 2. * PI / 3.), 1.;

  abcVector = dq0ToAbc * dq0Vector;

  return abcVector;
}
