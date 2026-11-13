/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator4OrderVBR.h>

#include <stdexcept>

using namespace CPS;

EMT::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR(
    const String &uid, const String &name, Logger::Level logLevel)
    : ReducedOrderSynchronGeneratorVBR(uid, name, logLevel),
      mEdq0_t(mAttributes->create<Matrix>("Edq0_t")) {

  // Keep the SG4 shaft damping consistent with the DP phasor implementation.
  setDamping(1.0);
  //
  mSGOrder = SGOrder::SG4Order;

  // model specific variables
  **mEdq0_t = Matrix::Zero(3, 1);
  mEhs_vbr = Matrix::Zero(3, 1);
}

EMT::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR(
    const String &name, Logger::Level logLevel)
    : SynchronGenerator4OrderVBR(name, name, logLevel) {}

EMT::Ph3::SynchronGenerator4OrderVBR::MetalDeviceData
EMT::Ph3::SynchronGenerator4OrderVBR::metalDeviceData() {
  if (!mModelAsNortonSource || mHasExciter || mHasPSS || mHasTurbineGovernor ||
      mHasGovernorAndTurbine)
    throw std::logic_error(
        "Metal SG4 VBR export supports only controller-free Norton models.");

  MetalDeviceData data{};
  for (UInt i = 0; i < 3; ++i) {
    data.voltageDq0[i] = (**mVdq0)(i, 0);
    data.currentDq0[i] = (**mIdq0)(i, 0);
    data.transientVoltageDq0[i] = (**mEdq0_t)(i, 0);
    data.matrixNodeIndices[i] = matrixNodeIndex(0, i);
  }
  for (UInt row = 0; row < 3; ++row)
    for (UInt col = 0; col < 3; ++col) {
      data.conductanceDq0[row * 3 + col] = mConductanceMatrixDq0(row, col);
    }
  // During DPsim initialize() the first component pre-step has not run yet,
  // therefore mConductanceMatrix can still be zero. Export the exact stamp
  // that calculateResistanceMatrix() will create on the first SG4 pre-step.
  const MatrixFixedSize<3, 3> abcToDq = get_parkTransformMatrix();
  const MatrixFixedSize<3, 3> dqToAbc = get_inverseParkTransformMatrix();
  const MatrixFixedSize<3, 3> conductanceAbc =
      dqToAbc * mConductanceMatrixDq0 * abcToDq;
  for (UInt row = 0; row < 3; ++row)
    for (UInt col = 0; col < 3; ++col)
      data.conductanceAbc[row * 3 + col] = conductanceAbc(row, col);

  data.timeStep = mTimeStep;
  data.baseVoltage = mBase_V;
  data.baseCurrent = mBase_I;
  data.baseImpedance = mBase_Z;
  data.baseOmegaElec = mBase_OmElec;
  data.baseOmegaMech = mBase_OmMech;
  data.inertia = mH;
  data.damping = mDamping;
  data.ldTransient = mLd_t;
  data.lqTransient = mLq_t;
  data.adTransient = mAd_t;
  data.bdTransient = mBd_t;
  data.aqTransient = mAq_t;
  data.bqTransient = mBq_t;
  data.dqTransient = mDq_t;
  data.fieldVoltage = **mEf;
  data.previousFieldVoltage = mEf_prev;
  data.mechanicalTorque = **mMechTorque;
  data.previousMechanicalTorque = mMechTorque_prev;
  data.electricalTorque = **mElecTorque;
  data.rotorSpeed = **mOmMech;
  data.rotorAngle = **mThetaMech;
  data.loadAngle = **mDelta;
  return data;
}

void EMT::Ph3::SynchronGenerator4OrderVBR::specificInitialization() {
  // initial voltage behind the transient reactance in the dq0 reference frame
  (**mEdq0_t)(0, 0) = (**mVdq0)(0, 0) - (**mIdq0)(1, 0) * mLq_t;
  (**mEdq0_t)(1, 0) = (**mVdq0)(1, 0) + (**mIdq0)(0, 0) * mLd_t;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Model specific initialization  ---"
                     "\nSG model: 4th order"
                     "\nInitial Ed_t (per unit): {:f}"
                     "\nInitial Eq_t (per unit): {:f}"
                     "\n--- Model specific initialization finished ---",

                     (**mEdq0_t)(0, 0), (**mEdq0_t)(1, 0));
  mSLog->flush();
}

void EMT::Ph3::SynchronGenerator4OrderVBR::stepInPerUnit() {

  const Matrix &idq0 = *mIdq0Value;
  const Matrix &vdq0 = *mVdq0Value;

  if (mSimTime > 0.0) {
    // calculate Edq_t at t=k
    (**mEdq0_t)(0, 0) = -idq0(1, 0) * mLq_t + vdq0(0, 0);
    (**mEdq0_t)(1, 0) = idq0(0, 0) * mLd_t + vdq0(1, 0);
    (**mEdq0_t)(2, 0) = 0.0;
  }

  updateParkTransformMatrices();

  // calculate resistance matrix at t=k+1
  calculateResistanceMatrix();

  // VBR history voltage
  mEhs_vbr(0, 0) = mAd_t * idq0(1, 0) + mBd_t * (**mEdq0_t)(0, 0);
  mEhs_vbr(1, 0) = mAq_t * idq0(0, 0) + mBq_t * (**mEdq0_t)(1, 0) +
                   mDq_t * mEf_prev + mDq_t * *mEfValue;
  mEhs_vbr(2, 0) = 0.0;

  // convert Edq_t into the abc reference frame
  mEvbr = mDq0ToAbc * mEhs_vbr * mBase_V;
}

bool EMT::Ph3::SynchronGenerator4OrderVBR::supportsHipBatch() noexcept {
  return mModelAsNortonSource && !mHasExciter && !mHasPSS &&
         !mHasGovernorAndTurbine && !mHasTurbineGovernor &&
         terminalNotGrounded(0);
}

EMT::Ph3::HipSG4State EMT::Ph3::SynchronGenerator4OrderVBR::exportHipState() {
  if (!supportsHipBatch())
    throw std::invalid_argument(
        "HIP SG4 batch supports only ungrounded Norton generators without "
        "controllers");
  HipSG4State state;
  state.theta = **mThetaMech;
  state.delta = **mDelta;
  state.omega = **mOmMech;
  state.electricalTorque = **mElecTorque;
  state.mechanicalTorque = **mMechTorque;
  state.previousMechanicalTorque = mMechTorque_prev;
  state.fieldVoltage = **mEf;
  state.previousFieldVoltage = mEf_prev;
  state.timestep = mTimeStep;
  state.inertia = mH;
  state.damping = mDamping;
  state.baseOmegaMech = mBase_OmMech;
  state.baseVoltage = mBase_V;
  state.baseCurrent = mBase_I;
  state.lqTransient = mLq_t;
  state.ldTransient = mLd_t;
  state.adTransient = mAd_t;
  state.bdTransient = mBd_t;
  state.aqTransient = mAq_t;
  state.bqTransient = mBq_t;
  state.dqTransient = mDq_t;
  for (int index = 0; index < 3; ++index) {
    state.idq0[index] = (**mIdq0)(index, 0);
    state.vdq0[index] = (**mVdq0)(index, 0);
    state.edq0Transient[index] = (**mEdq0_t)(index, 0);
    state.busRows[index] = static_cast<int>(matrixNodeIndex(0, index));
  }
  for (int col = 0; col < 3; ++col)
    for (int row = 0; row < 3; ++row)
      state.conductanceDq0[col * 3 + row] = mConductanceMatrixDq0(row, col);
  state.grounded = 0;
  return state;
}

void EMT::Ph3::SynchronGenerator4OrderVBR::applyHipPreState(
    const HipSG4State &state, Real time) {
  if (mEvbr.size() != 3)
    mEvbr.resize(3, 1);
  if (mIvbr.size() != 3)
    mIvbr.resize(3, 1);
  mSimTime = time;
  **mThetaMech = state.theta;
  **mDelta = state.delta;
  **mOmMech = state.omega;
  **mElecTorque = state.electricalTorque;
  **mMechTorque = state.mechanicalTorque;
  mMechTorque_prev = state.previousMechanicalTorque;
  **mEf = state.fieldVoltage;
  mEf_prev = state.previousFieldVoltage;
  for (int index = 0; index < 3; ++index) {
    (**mEdq0_t)(index, 0) = state.edq0Transient[index];
    mEhs_vbr(index, 0) = state.historyDq0[index];
    mEvbr(index, 0) = state.emfAbc[index];
    mIvbr(index, 0) = state.currentSourceAbc[index];
  }
  for (int col = 0; col < 3; ++col)
    for (int row = 0; row < 3; ++row)
      mConductanceMatrix(row, col) = state.conductanceAbc[col * 3 + row];
  (**mRightVector).setZero();
  if (!state.grounded)
    for (int phase = 0; phase < 3; ++phase)
      (**mRightVector)(state.busRows[phase], 0) = state.currentSourceAbc[phase];
}

void EMT::Ph3::SynchronGenerator4OrderVBR::applyHipPostState(
    const HipSG4State &state) {
  // Keep host-visible machine attributes current when a caller requests a
  // synchronized device state (for example, the optional hiplog trace).
  **mThetaMech = state.theta;
  **mDelta = state.delta;
  **mOmMech = state.omega;
  **mElecTorque = state.electricalTorque;
  **mMechTorque = state.mechanicalTorque;
  mMechTorque_prev = state.previousMechanicalTorque;
  **mEf = state.fieldVoltage;
  mEf_prev = state.previousFieldVoltage;
  for (int index = 0; index < 3; ++index) {
    (**mIntfVoltage)(index, 0) = state.terminalVoltageAbc[index];
    (**mIntfCurrent)(index, 0) = state.terminalCurrentAbc[index];
    (**mVdq0)(index, 0) = state.voltageDq0[index];
    (**mIdq0)(index, 0) = state.currentDq0[index];
  }
}
