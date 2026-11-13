/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_SynchronGenerator4OrderVBR.h>

using namespace CPS;

DP::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR(
    String uid, String name, Logger::Level logLevel)
    : SynchronGeneratorDQ(uid, name, logLevel),
      mAdmittance(MatrixComp::Zero(3, 3)),
      mInternalVoltage(MatrixComp::Zero(3, 1)),
      mNortonCurrent(MatrixComp::Zero(3, 1)),
      mVoltageDifference(MatrixComp::Zero(3, 1)) {}

DP::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR(
    String name, Logger::Level logLevel)
    : SynchronGenerator4OrderVBR(name, name, logLevel) {}

void DP::Ph3::SynchronGenerator4OrderVBR::setParametersOperationalPerUnit(
    Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
    Real Rs, Real Ld, Real Lq, Real Ld_t, Real Lq_t, Real Ll, Real Td0_t,
    Real Tq0_t, Real inertia) {
  setBaseParameters(nomPower, nomVolt, nomFreq, nomFieldCur);
  setOperationalPerUnitParameters(poleNumber, inertia, Rs, Ld, Lq, Ll, Ld_t,
                                  Lq_t, 0.8 * Ld_t, 0.8 * Lq_t, Td0_t, Tq0_t,
                                  0.05 * Td0_t, 0.05 * Tq0_t);
  calculateFundamentalFromOperationalParameters();
}

void DP::Ph3::SynchronGenerator4OrderVBR::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;

  // Reuse the established power-flow initialization for the rotor angle,
  // speed and d/q terminal quantities. The winding-level state-space model
  // is not used by this four-state VBR implementation.
  initializeMatrixAndStates();
  const Real xTransient = 0.5 * (**mLd_t + **mLq_t);
  mEdTransient = mVdq0(0, 0) - mIdq0(1, 0) * **mLq_t;
  mEqTransient = mVdq0(1, 0) + mIdq0(0, 0) * **mLd_t;
  mInitialEdTransient = mEdTransient;
  mInitialEqTransient = mEqTransient;
  mCurrentD = mIdq0(0, 0);
  mCurrentQ = mIdq0(1, 0);
  mInitialCurrentD = mCurrentD;
  mInitialCurrentQ = mCurrentQ;
  // Use the initialized terminal power as the mechanical reference. The
  // MATPOWER value is specified on the system base while the machine states
  // are on the machine base, so directly dividing it by mNomPower can create
  // a spurious initial acceleration when those bases differ.
  mMechanicalPower =
      -1.5 * (mVdq0(0, 0) * mIdq0(0, 0) + mVdq0(1, 0) * mIdq0(1, 0));

  const Complex phaseImpedance = mBase_Z * Complex(**mRs, xTransient);
  const Complex phaseAdmittance = Complex(1, 0) / phaseImpedance;
  mAdmittance.setZero();
  mAdmittance.diagonal().setConstant(phaseAdmittance);
  const MatrixComp nortonVoltage =
      **mIntfVoltage - phaseImpedance * **mIntfCurrent;
  MatrixComp nortonVoltageCopy = nortonVoltage;
  const Matrix nortonDq =
      abcToDq0Transform(mThetaMech, nortonVoltageCopy) / mBase_V;
  mNortonEd = nortonDq(0, 0);
  mNortonEq = nortonDq(1, 0);
  mTimeStep = timeStep;
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::SynchronGenerator4OrderVBR::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(mAdmittance, systemMatrix,
                                       matrixNodeIndex(0), matrixNodeIndex(0),
                                       false, terminalNotGrounded(0), mSLog);
}

void DP::Ph3::SynchronGenerator4OrderVBR::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  MatrixFixedSize<3, 1> dqInternal;
  dqInternal << mNortonEd + mEdTransient - mInitialEdTransient,
      mNortonEq + mEqTransient - mInitialEqTransient, 0;
  mInternalVoltage = mBase_V * dq0ToAbcTransform(mThetaMech, dqInternal);
  // The phase-domain Norton admittance is diagonal with the same value in
  // all three phases. Avoid dispatching a dense 3x3-by-3x1 product at every
  // RHS refresh.
  mNortonCurrent.noalias() = mAdmittance(0, 0) * mInternalVoltage;
  if (terminalNotGrounded(0)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             mNortonCurrent(phase, 0));
  }
}

void DP::Ph3::SynchronGenerator4OrderVBR::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (!terminalNotGrounded(0))
    return;
  const Matrix::Index imaginaryOffset = rightVector.rows() / 2;
  for (UInt phase = 0; phase < 3; ++phase) {
    const Matrix::Index row = matrixNodeIndex(0, phase);
    rightVector(row, 0) += mNortonCurrent(phase, 0).real();
    rightVector(row + imaginaryOffset, 0) += mNortonCurrent(phase, 0).imag();
  }
}

void DP::Ph3::SynchronGenerator4OrderVBR::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfVoltage);
}

void DP::Ph3::SynchronGenerator4OrderVBR::mnaCompPreStep(Real time,
                                                         Int timeStepCount) {
  updateCurrentAndStates(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::SynchronGenerator4OrderVBR::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  SynchronGeneratorDQ::mnaCompPostStep(time, timeStepCount, leftVector);
  if (!mInitialMechanicalPowerSet) {
    // Calibrate mechanical input from the solved initial network port. This
    // makes the first dynamic step torque-balanced even when the machine and
    // MATPOWER use different power bases or the PF voltage has rounding.
    mVoltageDifference = **mIntfVoltage - mInternalVoltage;
    **mIntfCurrent = mAdmittance(0, 0) * mVoltageDifference;
    MatrixFixedSize<3, 1> dqVoltage =
        abcToDq0Transform(mThetaMech, **mIntfVoltage) / mBase_V;
    MatrixFixedSize<3, 1> dqCurrent =
        abcToDq0Transform(mThetaMech, **mIntfCurrent) / mBase_I;
    mMechanicalPower = -1.5 * (dqVoltage(0, 0) * dqCurrent(0, 0) +
                               dqVoltage(1, 0) * dqCurrent(1, 0));
    // The solved PF port is authoritative. Anchor the transient current
    // states to it rather than to the analytic machine-base reconstruction;
    // this removes the small PF/phasor conversion residual at t=0.
    mInitialCurrentD = dqCurrent(0, 0);
    mInitialCurrentQ = dqCurrent(1, 0);
    mCurrentD = mInitialCurrentD;
    mCurrentQ = mInitialCurrentQ;
    mInitialMechanicalPowerSet = true;
  }
}

void DP::Ph3::SynchronGenerator4OrderVBR::updateCurrentAndStates(Real time) {
  // Reconstruct the component current from the implicit Norton equivalent and
  // transform it into the rotor frame for the machine equations.
  mVoltageDifference = **mIntfVoltage - mInternalVoltage;
  **mIntfCurrent = mAdmittance(0, 0) * mVoltageDifference;
  MatrixFixedSize<3, 1> dqVoltage =
      abcToDq0Transform(mThetaMech, **mIntfVoltage) / mBase_V;
  MatrixFixedSize<3, 1> dqCurrent =
      abcToDq0Transform(mThetaMech, **mIntfCurrent) / mBase_I;
  mCurrentD = dqCurrent(0, 0);
  mCurrentQ = dqCurrent(1, 0);

  const Real electricalPower =
      -1.5 * (dqVoltage(0, 0) * mCurrentD + dqVoltage(1, 0) * mCurrentQ);
  const Real omega = **mOmMech;
  if (time > 0) {
    // DP component current is positive into the machine; the classical
    // generator equations use current positive out of the machine.
    const Real dEd = (-(mEdTransient - mInitialEdTransient) -
                      (**mLq - **mLq_t) * (mCurrentQ - mInitialCurrentQ)) /
                     std::max(**mTq0_t, 1e-6);
    const Real dEq = (-(mEqTransient - mInitialEqTransient) +
                      (**mLd - **mLd_t) * (mCurrentD - mInitialCurrentD)) /
                     std::max(**mTd0_t, 1e-6);
    const Real dOmega =
        (mMechanicalPower - electricalPower - mDamping * (omega - 1.0)) /
        (2.0 * std::max(**mInertia, 1e-6));
    mEdTransient += mTimeStep * dEd;
    mEqTransient += mTimeStep * dEq;
    **mOmMech = omega + mTimeStep * dOmega;
    mThetaMech += mTimeStep * (omega - 1.0) * mBase_OmMech;
  }
}
