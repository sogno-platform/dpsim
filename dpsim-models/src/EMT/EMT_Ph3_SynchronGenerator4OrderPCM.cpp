/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator4OrderPCM.h>

using namespace CPS;

namespace {
MatrixFixedSize<3, 1> parkTransformFixed(Real theta, const Matrix &abcVector) {
  MatrixFixedSize<3, 3> abcToDq0;
  MatrixFixedSize<3, 1> abc;
  abc << abcVector(0, 0), abcVector(1, 0), abcVector(2, 0);

  const Real cosTheta = std::cos(theta);
  const Real sinTheta = std::sin(theta);
  constexpr Real sqrtThreeOverTwo = 0.86602540378443864676;
  const Real cosMinus120 = -0.5 * cosTheta + sqrtThreeOverTwo * sinTheta;
  const Real sinMinus120 = -0.5 * sinTheta - sqrtThreeOverTwo * cosTheta;
  const Real cosPlus120 = -0.5 * cosTheta - sqrtThreeOverTwo * sinTheta;
  const Real sinPlus120 = -0.5 * sinTheta + sqrtThreeOverTwo * cosTheta;

  abcToDq0 << 2. / 3. * cosTheta, 2. / 3. * cosMinus120, 2. / 3. * cosPlus120,
      -2. / 3. * sinTheta, -2. / 3. * sinMinus120, -2. / 3. * sinPlus120,
      1. / 3., 1. / 3., 1. / 3.;

  return abcToDq0 * abc;
}

MatrixFixedSize<3, 1> inverseParkTransformFixed(Real theta,
                                                const Matrix &dq0Vector) {
  MatrixFixedSize<3, 3> dq0ToAbc;
  MatrixFixedSize<3, 1> dq0;
  dq0 << dq0Vector(0, 0), dq0Vector(1, 0), dq0Vector(2, 0);

  const Real cosTheta = std::cos(theta);
  const Real sinTheta = std::sin(theta);
  constexpr Real sqrtThreeOverTwo = 0.86602540378443864676;
  const Real cosMinus120 = -0.5 * cosTheta + sqrtThreeOverTwo * sinTheta;
  const Real sinMinus120 = -0.5 * sinTheta - sqrtThreeOverTwo * cosTheta;
  const Real cosPlus120 = -0.5 * cosTheta - sqrtThreeOverTwo * sinTheta;
  const Real sinPlus120 = -0.5 * sinTheta + sqrtThreeOverTwo * cosTheta;

  dq0ToAbc << cosTheta, -sinTheta, 1., cosMinus120, -sinMinus120, 1.,
      cosPlus120, -sinPlus120, 1.;

  return dq0ToAbc * dq0;
}
} // namespace

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
  **mIntfCurrent = inverseParkTransformFixed(**mThetaMech, **mIdq0);
  **mIntfCurrent = **mIntfCurrent * mBase_I;

  // The current equations have voltage Jacobian
  // d(Id, Iq) / d(Vd, Vq) = [[0, -1/Ld_t], [1/Lq_t, 0]]. Stamp its
  // negative in MNA so that the source-side compensation cancels this
  // voltage dependence in the network iteration. The transform depends on
  // rotor position and therefore the matrix is updated every step.
  Matrix dq0ToAbc(3, 3);
  Matrix abcToDq0(3, 3);
  const Real theta = **mThetaMech;
  dq0ToAbc << cos(theta), -sin(theta), 1., cos(theta - 2. * PI / 3.),
      -sin(theta - 2. * PI / 3.), 1., cos(theta + 2. * PI / 3.),
      -sin(theta + 2. * PI / 3.), 1.;
  abcToDq0 << 2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * PI / 3.),
      2. / 3. * cos(theta + 2. * PI / 3.), -2. / 3. * sin(theta),
      -2. / 3. * sin(theta - 2. * PI / 3.),
      -2. / 3. * sin(theta + 2. * PI / 3.), 1. / 3., 1. / 3., 1. / 3.;
  Matrix dq0Conductance = Matrix::Zero(3, 3);
  dq0Conductance(0, 1) = 1.0 / mLd_t;
  dq0Conductance(1, 0) = -1.0 / mLq_t;
  mNortonConductance = dq0ToAbc * dq0Conductance * abcToDq0 / mBase_Z;
}

void EMT::Ph3::SynchronGenerator4OrderPCM::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  const Matrix compensationCurrent = mNortonConductance * **mIntfVoltage;
  Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                         (**mIntfCurrent)(0, 0) + compensationCurrent(0, 0));
  Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                         (**mIntfCurrent)(1, 0) + compensationCurrent(1, 0));
  Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                         (**mIntfCurrent)(2, 0) + compensationCurrent(2, 0));
}

void EMT::Ph3::SynchronGenerator4OrderPCM::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (!terminalNotGrounded(0))
    return;

  for (UInt row = 0; row < 3; ++row) {
    for (UInt col = 0; col < 3; ++col) {
      Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, row),
                               matrixNodeIndex(0, col),
                               mNortonConductance(row, col));
    }
  }
}

void EMT::Ph3::SynchronGenerator4OrderPCM::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  Base::ReducedOrderSynchronGenerator<Real>::mnaCompInitialize(omega, timeStep,
                                                               leftVector);
  for (UInt row = 0; row < 3; ++row)
    for (UInt col = 0; col < 3; ++col)
      mVariableSystemMatrixEntries.emplace_back(matrixNodeIndex(0, row),
                                                matrixNodeIndex(0, col));
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
  **mIntfCurrent = inverseParkTransformFixed(**mThetaMech, **mIdq0);
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
  **mVdq0 = parkTransformFixed(**mThetaMech, **mIntfVoltage);
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
    if (Math::abs((**mVdq0)(0, 0) - mVdq0PrevIter(0, 0)) > mTolerance ||
        Math::abs((**mVdq0)(1, 0) - mVdq0PrevIter(1, 0)) > mTolerance)
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
  return Matrix(parkTransformFixed(theta, abcVector));
}

Matrix EMT::Ph3::SynchronGenerator4OrderPCM::inverseParkTransform(
    Real theta, const Matrix &dq0Vector) {
  return Matrix(inverseParkTransformFixed(theta, dq0Vector));
}
