/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h>

using namespace CPS;

template <>
Base::ReducedOrderSynchronGenerator<Real>::ReducedOrderSynchronGenerator(
    String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mVdq0(mAttributes->create<Matrix>("Vdq0")),
      mIdq0(mAttributes->create<Matrix>("Idq0")),
      mElecTorque(mAttributes->create<Real>("Te")),
      mMechTorque(mAttributes->create<Real>("Tm")),
      mOmMech(mAttributes->create<Real>("w_r")),
      mThetaMech(mAttributes->create<Real>("Theta")),
      mDelta(mAttributes->create<Real>("delta")),
      mEf(mAttributes->create<Real>("Ef")) {

  mSimTime = 0.0;

  // declare state variables
  **mVdq0 = Matrix::Zero(3, 1);
  **mIdq0 = Matrix::Zero(3, 1);

  // default model is Norton equivalent
  mModelAsNortonSource = true;
  SPDLOG_LOGGER_DEBUG(this->mSLog,
                      "SG per default modelled as Norton equivalent");
}

template <>
Base::ReducedOrderSynchronGenerator<Complex>::ReducedOrderSynchronGenerator(
    String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mVdq(mAttributes->create<Matrix>("Vdq0")),
      mIdq(mAttributes->create<Matrix>("Idq0")),
      mElecTorque(mAttributes->create<Real>("Te")),
      mMechTorque(mAttributes->create<Real>("Tm")),
      mOmMech(mAttributes->create<Real>("w_r")),
      mThetaMech(mAttributes->create<Real>("Theta")),
      mDelta(mAttributes->create<Real>("delta")),
      mEf(mAttributes->create<Real>("Ef")) {

  mSimTime = 0.0;

  // declare state variables
  ///FIXME: The mVdq0 and mVdq member variables are mutually exclusive and carry the same attribute name. Maybe they can be unified?
  **mVdq = Matrix::Zero(2, 1);
  **mIdq = Matrix::Zero(2, 1);

  // default model is Norton equivalent
  mModelAsNortonSource = true;
  SPDLOG_LOGGER_DEBUG(this->mSLog,
                      "SG per default modelled as Norton equivalent");
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::setModelAsNortonSource(
    Bool modelAsCurrentSource) {
  mModelAsNortonSource = modelAsCurrentSource;

  if (mModelAsNortonSource) {
    this->setVirtualNodeNumber(0);
    SPDLOG_LOGGER_DEBUG(this->mSLog, "Setting SG model to Norton equivalent");
  } else {
    this->setVirtualNodeNumber(2);
    SPDLOG_LOGGER_DEBUG(this->mSLog, "Setting SG model to Thevenin equivalent");
  }
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::setBaseParameters(
    Real nomPower, Real nomVolt, Real nomFreq) {

  /// used p.u. system: Lad-Base reciprocal per unit system (Kundur, p. 84-88)

  // set base nominal values
  mNomPower = nomPower;
  mNomVolt = nomVolt;
  mNomFreq = nomFreq;
  mNomOmega = nomFreq * 2 * PI;

  // Set base stator values
  mBase_V_RMS = mNomVolt;
  mBase_V = mNomVolt / sqrt(3) * sqrt(2);
  mBase_I_RMS = mNomPower / mBase_V_RMS;
  mBase_I = mNomPower / ((3. / 2.) * mBase_V);
  mBase_Z = mBase_V_RMS / mBase_I_RMS;
  mBase_OmElec = mNomOmega;
  mBase_OmMech = mBase_OmElec;
  mBase_L = mBase_Z / mBase_OmElec;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<
    VarType>::setOperationalParametersPerUnit(Real nomPower, Real nomVolt,
                                              Real nomFreq, Real H, Real Ld,
                                              Real Lq, Real L0, Real Ld_t,
                                              Real Td0_t) {

  setBaseParameters(nomPower, nomVolt, nomFreq);

  mLd = Ld;
  mLq = Lq;
  mL0 = L0;
  mLd_t = Ld_t;
  mTd0_t = Td0_t;
  mH = H;

  SPDLOG_LOGGER_INFO(this->mSLog,
                     "Set base parameters: \n"
                     "nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n",
                     mNomPower, mNomVolt, mNomFreq);

  SPDLOG_LOGGER_INFO(this->mSLog,
                     "Set operational parameters in per unit: \n"
                     "inertia: {:e}\n"
                     "Ld: {:e}\nLq: {:e}\nL0: {:e}\n"
                     "Ld_t: {:e}\nTd0_t: {:e}\n",
                     mH, mLd, mLq, mL0, mLd_t, mTd0_t);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<
    VarType>::setOperationalParametersPerUnit(Real nomPower, Real nomVolt,
                                              Real nomFreq, Real H, Real Ld,
                                              Real Lq, Real L0, Real Ld_t,
                                              Real Lq_t, Real Td0_t,
                                              Real Tq0_t) {

  setBaseParameters(nomPower, nomVolt, nomFreq);

  mLd = Ld;
  mLq = Lq;
  mL0 = L0;
  mLd_t = Ld_t;
  mLq_t = Lq_t;
  mTd0_t = Td0_t;
  mTq0_t = Tq0_t;
  mH = H;

  SPDLOG_LOGGER_INFO(this->mSLog,
                     "Set base parameters: \n"
                     "nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n",
                     mNomPower, mNomVolt, mNomFreq);

  SPDLOG_LOGGER_INFO(this->mSLog,
                     "Set operational parameters in per unit: \n"
                     "inertia: {:e}\n"
                     "Ld: {:e}\nLq: {:e}\nL0: {:e}\n"
                     "Ld_t: {:e}\nLq_t: {:e}\n"
                     "Td0_t: {:e}\nTq0_t: {:e}\n",
                     mH, mLd, mLq, mL0, mLd_t, mLq_t, mTd0_t, mTq0_t);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<
    VarType>::setOperationalParametersPerUnit(Real nomPower, Real nomVolt,
                                              Real nomFreq, Real H, Real Ld,
                                              Real Lq, Real L0, Real Ld_t,
                                              Real Lq_t, Real Td0_t, Real Tq0_t,
                                              Real Ld_s, Real Lq_s, Real Td0_s,
                                              Real Tq0_s, Real Taa) {

  setBaseParameters(nomPower, nomVolt, nomFreq);

  mLd = Ld;
  mLq = Lq;
  mL0 = L0;
  mLd_t = Ld_t;
  mLq_t = Lq_t;
  mLd_s = Ld_s;
  mLq_s = Lq_s;
  mTd0_t = Td0_t;
  mTq0_t = Tq0_t;
  mTd0_s = Td0_s;
  mTq0_s = Tq0_s;
  mTaa = Taa;
  mH = H;

  SPDLOG_LOGGER_INFO(this->mSLog,
                     "Set base parameters: \n"
                     "nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n",
                     mNomPower, mNomVolt, mNomFreq);

  SPDLOG_LOGGER_INFO(this->mSLog,
                     "Set operational parameters in per unit: \n"
                     "inertia: {:e}\n"
                     "Ld: {:e}\nLq: {:e}\nL0: {:e}\n"
                     "Ld_t: {:e}\nLq_t: {:e}\n"
                     "Td0_t: {:e}\nTq0_t: {:e}\n"
                     "Ld_s: {:e}\nLq_s: {:e}\n"
                     "Td0_s: {:e}\nTq0_s: {:e}\n"
                     "Taa: {:e}\n",
                     mH, mLd, mLq, mL0, mLd_t, mLq_t, mTd0_t, mTq0_t, mLd_s,
                     mLq_s, mTd0_s, mTq0_s, mTaa);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::scaleInertiaConstant(
    Real scalingFactor) {
  mH = mH * scalingFactor;
  SPDLOG_LOGGER_INFO(
      this->mSLog,
      "Scaling inertia with factor {:e}:\n resulting inertia: {:e}\n",
      scalingFactor, mH);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::calculateVBRconstants() {

  Real Tf = 0;
  if (mSGOrder == SGOrder::SG5Order) {
    mYd = (mTd0_s / mTd0_t) * (mLd_s / mLd_t) * (mLd - mLd_t);
    mYq = 0.0;
    Tf = mTaa / mTd0_t;
  } else if (mSGOrder == SGOrder::SG6aOrder) {
    mYd = (mTd0_s / mTd0_t) * (mLd_s / mLd_t) * (mLd - mLd_t);
    mYq = (mTq0_s / mTq0_t) * (mLq_s / mLq_t) * (mLq - mLq_t);
    Tf = mTaa / mTd0_t;
  } else {
    mYd = 0;
    mYq = 0;
  }

  Real Zq_t = mLd - mLd_t - mYd;
  Real Zd_t = mLq - mLq_t - mYq;
  Real Zq_s = mLd_t - mLd_s + mYd;
  Real Zd_s = mLq_t - mLq_s + mYq;

  mAd_t = mTimeStep * Zd_t / (2 * mTq0_t + mTimeStep);
  mBd_t = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);
  mAq_t = -mTimeStep * Zq_t / (2 * mTd0_t + mTimeStep);
  mBq_t = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
  mDq_t = mTimeStep * (1 - Tf) / (2 * mTd0_t + mTimeStep);

  if (mSGOrder == SGOrder::SG5Order) {
    mAd_s = (mTimeStep * (mLq - mLq_s)) / (2 * mTq0_s + mTimeStep);
    mCd_s = (2 * mTq0_s - mTimeStep) / (2 * mTq0_s + mTimeStep);
    mAq_s = (-mTimeStep * Zq_s + mTimeStep * mAq_t) / (2 * mTd0_s + mTimeStep);
    mBq_s = (mTimeStep * mBq_t + mTimeStep) / (2 * mTd0_s + mTimeStep);
    mCq_s = (2 * mTd0_s - mTimeStep) / (2 * mTd0_s + mTimeStep);
    mDq_s = (mTimeStep * mDq_t + Tf * mTimeStep) / (2 * mTd0_s + mTimeStep);
  } else if (mSGOrder == SGOrder::SG6aOrder || mSGOrder == SGOrder::SG6bOrder) {
    mAd_s = (mTimeStep * Zd_s + mTimeStep * mAd_t) / (2 * mTq0_s + mTimeStep);
    mBd_s = (mTimeStep * mBd_t + mTimeStep) / (2 * mTq0_s + mTimeStep);
    mCd_s = (2 * mTq0_s - mTimeStep) / (2 * mTq0_s + mTimeStep);
    mAq_s = (-mTimeStep * Zq_s + mTimeStep * mAq_t) / (2 * mTd0_s + mTimeStep);
    mBq_s = (mTimeStep * mBq_t + mTimeStep) / (2 * mTd0_s + mTimeStep);
    mCq_s = (2 * mTd0_s - mTimeStep) / (2 * mTd0_s + mTimeStep);
    mDq_s = (mTimeStep * mDq_t + Tf * mTimeStep) / (2 * mTd0_s + mTimeStep);
  }
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<
    VarType>::calculateResistanceMatrixConstants() {
  if (mSGOrder == SGOrder::SG3Order) {
    mA = -mLq;
    mB = mLd_t - mAq_t;
  }
  if (mSGOrder == SGOrder::SG4Order) {
    mA = -mAd_t - mLq_t;
    mB = mLd_t - mAq_t;
  }
  if (mSGOrder == SGOrder::SG5Order || mSGOrder == SGOrder::SG6aOrder ||
      mSGOrder == SGOrder::SG6bOrder) {
    mA = -mLq_s - mAd_s;
    mB = mLd_s - mAq_s;
  }
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::setInitialValues(
    Complex initComplexElectricalPower, Real initMechanicalPower,
    Complex initTerminalVoltage) {

  mInitElecPower = initComplexElectricalPower;
  mInitMechPower = initMechanicalPower;

  mInitVoltage = initTerminalVoltage;
  mInitVoltageAngle = Math::phase(mInitVoltage);

  mInitCurrent = std::conj(mInitElecPower / mInitVoltage);
  mInitCurrentAngle = Math::phase(mInitCurrent);

  mInitVoltage = mInitVoltage / mBase_V_RMS;
  mInitCurrent = mInitCurrent / mBase_I_RMS;

  mInitialValuesSet = true;

  SPDLOG_LOGGER_DEBUG(this->mSLog,
                      "\n--- Set initial values  ---"
                      "\nInitial active power: {:}W = {:} p.u."
                      "\nInitial reactive power W: {:}W = {:} p.u."
                      "\nInitial terminal voltage magnitude: {:} p.u."
                      "\nInitial terminal voltage phase: {:} rad = ({:}°)"
                      "\nInitial current magnitude: {:} p.u."
                      "\nInitial current phase: {:} rad = ({:}°)"
                      "\n--- Set initial values finished ---\n",
                      mInitElecPower.real(), mInitElecPower.real() / mNomPower,
                      mInitElecPower.imag(), mInitElecPower.imag() / mNomPower,
                      Math::abs(mInitVoltage), Math::phase(mInitVoltage),
                      Math::phaseDeg(mInitVoltage), Math::abs(mInitCurrent),
                      Math::phase(mInitCurrent), Math::phaseDeg(mInitCurrent));
  this->mSLog->flush();
}

template <>
void Base::ReducedOrderSynchronGenerator<Real>::initializeFromNodesAndTerminals(
    Real frequency) {

  this->updateMatrixNodeIndices();

  if (!mInitialValuesSet)
    this->setInitialValues(-this->terminal(0)->singlePower(),
                           -this->terminal(0)->singlePower().real(),
                           this->initialSingleVoltage(0));

  // Initialize mechanical torque
  **mMechTorque = mInitMechPower / mNomPower;
  mMechTorque_prev = **mMechTorque;

  // calculate steady state machine emf (i.e. voltage behind synchronous reactance)
  Complex Eq0 = mInitVoltage + Complex(0, mLq) * mInitCurrent;

  // Load angle
  **mDelta = Math::phase(Eq0);

  // convert currrents to dq reference frame
  (**mIdq0)(0, 0) = Math::abs(mInitCurrent) * sin(**mDelta - mInitCurrentAngle);
  (**mIdq0)(1, 0) = Math::abs(mInitCurrent) * cos(**mDelta - mInitCurrentAngle);

  // convert voltages to dq reference frame
  (**mVdq0)(0, 0) = Math::abs(mInitVoltage) * sin(**mDelta - mInitVoltageAngle);
  (**mVdq0)(1, 0) = Math::abs(mInitVoltage) * cos(**mDelta - mInitVoltageAngle);

  // calculate Ef
  **mEf = Math::abs(Eq0) + (mLd - mLq) * (**mIdq0)(0, 0);
  mEf_prev = **mEf;

  // initial electrical torque
  **mElecTorque =
      (**mVdq0)(0, 0) * (**mIdq0)(0, 0) + (**mVdq0)(1, 0) * (**mIdq0)(1, 0);

  // Initialize omega mech with nominal system frequency
  **mOmMech = mNomOmega / mBase_OmMech;

  // initialize theta and calculate transform matrix
  **mThetaMech = **mDelta - PI / 2.;

  // Initialize controllers
  if (mHasPSS)
    mPSS->initialize(**mOmMech, **mElecTorque, (**mVdq0)(0, 0),
                     (**mVdq0)(1, 0));
  if (mHasExciter)
    mExciter->initialize(Math::abs(mInitVoltage), **mEf);
  if (mHasTurbineGovernor)
    mTurbineGovernor->initialize(**mMechTorque);

  // set initial interface current
  (**mIntfCurrent)(0, 0) = (mInitCurrent * mBase_I).real();
  (**mIntfCurrent)(1, 0) = (mInitCurrent * mBase_I * SHIFT_TO_PHASE_B).real();
  (**mIntfCurrent)(2, 0) = (mInitCurrent * mBase_I * SHIFT_TO_PHASE_C).real();

  // set initial interface voltage
  (**mIntfVoltage)(0, 0) = (mInitVoltage * mBase_V).real();
  (**mIntfVoltage)(1, 0) = (mInitVoltage * mBase_V * SHIFT_TO_PHASE_B).real();
  (**mIntfVoltage)(2, 0) = (mInitVoltage * mBase_V * SHIFT_TO_PHASE_C).real();

  SPDLOG_LOGGER_DEBUG(this->mSLog,
                      "\n--- Initialization from power flow  ---"
                      "\nInitial Vd (per unit): {:f}"
                      "\nInitial Vq (per unit): {:f}"
                      "\nInitial Id (per unit): {:f}"
                      "\nInitial Iq (per unit): {:f}"
                      "\nInitial Ef (per unit): {:f}"
                      "\nInitial mechanical torque (per unit): {:f}"
                      "\nInitial electrical torque (per unit): {:f}"
                      "\nInitial initial mechanical theta (per unit): {:f}"
                      "\nInitial delta (per unit): {:f} (= {:f}°)"
                      "\n--- Initialization from power flow finished ---",

                      (**mVdq0)(0, 0), (**mVdq0)(1, 0), (**mIdq0)(0, 0),
                      (**mIdq0)(1, 0), **mEf, **mMechTorque, **mElecTorque,
                      **mThetaMech, **mDelta, **mDelta * 180 / PI);
  this->mSLog->flush();
}

template <>
void Base::ReducedOrderSynchronGenerator<
    Complex>::initializeFromNodesAndTerminals(Real frequency) {

  this->updateMatrixNodeIndices();

  if (!mInitialValuesSet)
    this->setInitialValues(-this->terminal(0)->singlePower(),
                           -this->terminal(0)->singlePower().real(),
                           this->initialSingleVoltage(0));

  // Initialize mechanical torque
  **mMechTorque = mInitMechPower / mNomPower;
  mMechTorque_prev = **mMechTorque;

  // calculate steady state machine emf (i.e. voltage behind synchronous reactance)
  Complex Eq0 = mInitVoltage + Complex(0, mLq) * mInitCurrent;

  // Load angle
  **mDelta = Math::phase(Eq0);

  // convert currrents to dq reference frame
  (**mIdq)(0, 0) = Math::abs(mInitCurrent) * sin(**mDelta - mInitCurrentAngle);
  (**mIdq)(1, 0) = Math::abs(mInitCurrent) * cos(**mDelta - mInitCurrentAngle);

  // convert voltages to dq reference frame
  (**mVdq)(0, 0) = Math::abs(mInitVoltage) * sin(**mDelta - mInitVoltageAngle);
  (**mVdq)(1, 0) = Math::abs(mInitVoltage) * cos(**mDelta - mInitVoltageAngle);

  // calculate Ef
  **mEf = Math::abs(Eq0) + (mLd - mLq) * (**mIdq)(0, 0);
  mEf_prev = **mEf;

  // initial electrical torque
  **mElecTorque =
      (**mVdq)(0, 0) * (**mIdq)(0, 0) + (**mVdq)(1, 0) * (**mIdq)(1, 0);

  // Initialize omega mech with nominal system frequency
  **mOmMech = mNomOmega / mBase_OmMech;

  // initialize theta and calculate transform matrix
  **mThetaMech = **mDelta - PI / 2.;

  // Initialize controllers
  if (mHasPSS)
    mPSS->initialize(**mOmMech, **mElecTorque, (**mVdq0)(0, 0),
                     (**mVdq0)(1, 0));
  if (mHasExciter)
    mExciter->initialize(Math::abs(mInitVoltage), **mEf);
  //if (mHasTurbine)
  //	mTurbine->initialize(**mMechTorque);
  if (mHasGovernor)
    mGovernor->initialize(**mMechTorque);
  //if (mHasTurbineGovernor){
  //	if(mSteam){
  //	mSteamTurbine->initialize(**mMechTorque);
  //	mSteamTurbineGovernor->initialize(**mMechTorque);
  //	}
  //	if(mHydro){
  //	mHydroTurbine->initialize(**mMechTorque);
  //	mHydroTurbineGovernor->initialize(**mMechTorque);
  //	}
  //}

  // set initial value of current
  (**mIntfCurrent)(0, 0) = mInitCurrent * mBase_I_RMS;

  // set initial interface voltage
  (**mIntfVoltage)(0, 0) = mInitVoltage * mBase_V_RMS;

  // set initial interface voltage
  (**mIntfVoltage)(0, 0) = mInitVoltage * mBase_V_RMS;

  SPDLOG_LOGGER_DEBUG(this->mSLog,
                      "\n--- Initialization from power flow  ---"
                      "\nInitial Vd (per unit): {:f}"
                      "\nInitial Vq (per unit): {:f}"
                      "\nInitial Id (per unit): {:f}"
                      "\nInitial Iq (per unit): {:f}"
                      "\nInitial Ef (per unit): {:f}"
                      "\nInitial mechanical torque (per unit): {:f}"
                      "\nInitial electrical torque (per unit): {:f}"
                      "\nInitial initial mechanical theta (per unit): {:f}"
                      "\nInitial delta (per unit): {:f} (= {:f}°)"
                      "\n--- Initialization from power flow finished ---",

                      (**mVdq)(0, 0), (**mVdq)(1, 0), (**mIdq)(0, 0),
                      (**mIdq)(1, 0), **mEf, **mMechTorque, **mElecTorque,
                      **mThetaMech, **mDelta, **mDelta * 180 / PI);
  this->mSLog->flush();
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

  this->updateMatrixNodeIndices();
  mTimeStep = timeStep;
  calculateVBRconstants();
  calculateResistanceMatrixConstants();
  initializeResistanceMatrix();
  specificInitialization();
}

template <>
void Base::ReducedOrderSynchronGenerator<Complex>::mnaCompPreStep(
    Real time, Int timeStepCount) {
  mSimTime = time;

  // update governor variables
  if (mHasTurbineGovernor) {
    mMechTorque_prev = **mMechTorque;
    **mMechTorque = mTurbineGovernor->step(**mOmMech, mTimeStep);
  }
  // update governor variables
  /*
	if (mHasTurbineGovernor) {
		if(mSteam){
		mMechTorque_prev = **mMechTorque;
		**mMechTorque = mSteamTurbine->step(mSteamTurbineGovernor->step(**mOmMech, mTimeStep), mTimeStep);
		}
		if(mHydro){
		mMechTorque_prev = **mMechTorque;
		**mMechTorque = mHydroTurbine->step(mHydroTurbineGovernor->step(**mOmMech, mTimeStep), mTimeStep);
		}
	}
	*/

  // calculate mechanical variables at t=k+1 with forward euler
  **mElecTorque =
      (**mVdq)(0, 0) * (**mIdq)(0, 0) + (**mVdq)(1, 0) * (**mIdq)(1, 0);
  **mOmMech = **mOmMech +
              mTimeStep * (1. / (2. * mH) * (mMechTorque_prev - **mElecTorque));
  **mThetaMech = **mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);
  **mDelta = **mDelta + mTimeStep * (**mOmMech - 1.) * mBase_OmMech;

  // update exciter and PSS variables
  if (mHasPSS)
    mVpss = mPSS->step(**mOmMech, **mElecTorque, (**mVdq)(0, 0), (**mVdq)(1, 0),
                       mTimeStep);
  if (mHasExciter) {
    mEf_prev = **mEf;
    **mEf = mExciter->step((**mVdq)(0, 0), (**mVdq)(1, 0), mTimeStep, mVpss);
  }

  stepInPerUnit();
  (**mRightVector).setZero();
  mnaApplyRightSideVectorStamp(**mRightVector);
}

template <>
void Base::ReducedOrderSynchronGenerator<Real>::mnaCompPreStep(
    Real time, Int timeStepCount) {
  mSimTime = time;

  // update governor variables
  /*
	if (mHasTurbineGovernor) {
		if(mSteam){
		mMechTorque_prev = **mMechTorque;
		**mMechTorque = mSteamTurbine->step(mSteamTurbineGovernor->step(**mOmMech, mTimeStep), mTimeStep);
		}
		if(mHydro){
		mMechTorque_prev = **mMechTorque;
		**mMechTorque = mHydroTurbine->step(mHydroTurbineGovernor->step(**mOmMech, mTimeStep), mTimeStep);
		}
	}
	*/

  // update governor variables
  if (mHasGovernor) {
    mMechTorque_prev = **mMechTorque;
    **mMechTorque = mGovernor->step(**mOmMech, mTimeStep);
  }

  // calculate mechanical variables at t=k+1 with forward euler
  **mElecTorque =
      ((**mVdq0)(0, 0) * (**mIdq0)(0, 0) + (**mVdq0)(1, 0) * (**mIdq0)(1, 0));
  **mOmMech = **mOmMech +
              mTimeStep * (1. / (2. * mH) * (mMechTorque_prev - **mElecTorque));
  **mThetaMech = **mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);
  **mDelta = **mDelta + mTimeStep * (**mOmMech - 1.) * mBase_OmMech;

  // update exciter and PSS variables
  if (mHasPSS)
    mVpss =
        mPSS->step(**mOmMech, **mElecTorque, (**mVdq)(0, 0), (**mVdq)(1, 0));
  if (mHasExciter) {
    mEf_prev = **mEf;
    **mEf = mExciter->step((**mVdq)(0, 0), (**mVdq)(1, 0), mTimeStep, mVpss);
  }

  stepInPerUnit();
  (**mRightVector).setZero();
  mnaApplyRightSideVectorStamp(**mRightVector);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::
    mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                  AttributeBase::List &attributeDependencies,
                                  AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfVoltage);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::mnaCompPreStep(
    Real time, Int timeStepCount) {
  mSimTime = time;
  stepInPerUnit();
  (**mRightVector).setZero();
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::
    mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompPostStep(**leftVector);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addExciter(
    std::shared_ptr<CPS::Base::ExciterParameters> exciterParameters,
    ExciterType exciterType) {

  if (exciterType == ExciterType::DC1)
    mExciter = CPS::Signal::ExciterDC1::make("Exciter_" + this->name(),
                                             this->mLogLevel);
  else if (exciterType == ExciterType::DC1Simp)
    mExciter = CPS::Signal::ExciterDC1Simp::make("Exciter_" + this->name(),
                                                 this->mLogLevel);
  else if (exciterType == ExciterType::ST1Simp)
    mExciter = CPS::Signal::ExciterST1Simp::make("Exciter_" + this->name(),
                                                 this->mLogLevel);

  mExciter->setParameters(exciterParameters);
  mHasExciter = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addExciter(
    std::shared_ptr<Base::Exciter> exciter) {
  mExciter = exciter;
  mHasExciter = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addPSS(
    Real Kp, Real Kv, Real Kw, Real T1, Real T2, Real T3, Real T4, Real Vs_max,
    Real Vs_min, Real Tw) {

  if (!mHasExciter) {
    std::cerr << "PSS can not be used without Exciter! PSS will be ignored!"
              << std::endl;
    SPDLOG_LOGGER_ERROR(
        this->mSLog,
        "PSS can not be used without Exciter! PSS will be ignored!");
    return;
  }

  mPSS = Signal::PSS1A::make(**this->mName + "_PSS", this->mLogLevel);
  mPSS->setParameters(Kp, Kv, Kw, T1, T2, T3, T4, Vs_max, Vs_min, Tw);
  mHasPSS = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addPSS(
    Real Kp, Real Kv, Real Kw, Real T1, Real T2, Real T3, Real T4, Real Vs_max,
    Real Vs_min, Real Tw) {

  if (!mHasExciter) {
    std::cerr << "PSS can not be used without Exciter! PSS will be ignored!"
              << std::endl;
    SPDLOG_LOGGER_ERROR(
        this->mSLog,
        "PSS can not be used without Exciter! PSS will be ignored!");
    return;
  }

  mPSS = Signal::PSS1A::make(**this->mName + "_PSS", this->mLogLevel);
  mPSS->setParameters(Kp, Kv, Kw, T1, T2, T3, T4, Vs_max, Vs_min, Tw);
  mHasPSS = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addPSS(
    std::shared_ptr<Signal::PSS1A> PSS) {

  if (!mHasExciter) {
    std::cerr << "PSS can not be used without Exciter! PSS will be ignored!"
              << std::endl;
    SPDLOG_LOGGER_ERROR(
        this->mSLog,
        "PSS can not be used without Exciter! PSS will be ignored!");
    return;
  }

  mPSS = PSS;
  mHasPSS = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addGovernor(
    std::shared_ptr<Base::Governor> governor) {
  //mTurbineGovernor = Signal::TurbineGovernorType1::make(**this->mName + "_TurbineGovernor", this->mLogLevel);
  //mTurbineGovernor->setParameters(T3, T4, T5, Tc, Ts, R, Pmin, Pmax, OmRef);
  //mHasTurbineGovernor = true;
  mGovernor = governor;
  mHasGovernor = true;
}

/*
void Base::ReducedOrderSynchronGenerator<VarType>::addSteamTurbine(Real Fhp, Real Fip, Real Flp, Real Tch, Real Trh, Real Tco, Real Pminit) {
	mSteam=true;
	mSteamTurbine = Signal::SteamTurbine::make(**this->mName + "_SteamTurbine", this->mLogLevel);
	mSteamTurbine->setParameters(Fhp,Fip,Flp,Tch,Trh,Tco);
	mSteamTurbine->initialize(Pminit);
	mHasTurbine = true;
	if (mHasGovernor)
	mHasTurbineGovernor=true;
}

//Create a steam turbine via exsiting object
template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addSteamTurbine(std::shared_ptr<Signal::SteamTurbine> steamTurbine) {
	mSteam=true;
	mSteamTurbine = steamTurbine;
	mHasTurbine = true;
	if (mHasGovernor)
	mHasTurbineGovernor=true;
}

void Base::ReducedOrderSynchronGenerator<VarType>::addSteamTurbineGovernor(Real OmRef, Real Pref, Real R, Real T2, Real T3, 
													Real dPmax, Real dPmin, Real Pmax, Real Pmin)
{
	mSteam=true;
	mSteamTurbineGovernor = Signal::SteamTurbineGovernor::make(**this->mName + "_SteamTurbineGovernor", this->mLogLevel);
	mSteamTurbineGovernor->setParameters(OmRef, R, T2, T3, dPmax, dPmin, Pmax, Pmin);
	mSteamTurbineGovernor->initialize(Pref);
	mHasGovernor = true;
	if (mHasTurbine)
	mHasTurbineGovernor=true;
}
//Create a steam turbine governor via exsiting object
template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addSteamTurbineGovernor(std::shared_ptr<Signal::SteamTurbineGovernor> steamTurbineGovernor)
{
	mSteam=true;
	mSteamTurbineGovernor = steamTurbineGovernor;
	mHasGovernor = true;
	if (mHasTurbine)
	mHasTurbineGovernor=true;
}



//Create a Hydro Turbine
template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addHydroTurbine(Real Tw, Real Pminit)
{
	mHydro=true;
	mHydroTurbine = Signal::HydroTurbine::make(**this->mName + "HydroTurbine", this->mLogLevel);
	mHydroTurbine->setParameters(Tw);
	mHydroTurbine->initialize(Pminit);
	mHasTurbine = true;
	if (mHasGovernor)
	mHasTurbineGovernor=true;
}
//Create a Hydro turbine via exsiting object
template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addHydroTurbine(std::shared_ptr<Signal::HydroTurbine> HydroTurbine)
{
	mHydro=true;
	mHydroTurbine = HydroTurbine;
	mHasTurbine = true;
	if (mHasGovernor)
	mHasTurbineGovernor=true;
}
//Create a Hydro Turbine Governor
template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addHydroTurbineGovernor(Real OmRef, Real Pref, Real R, Real T1, Real T2, Real T3,
                                         					Real Pmax, Real Pmin)
{
	mHydro=true;
	mHydroTurbineGovernor = Signal::HydroTurbineGovernor::make(**this->mName + "_HydroTurbineGovernor", this->mLogLevel);
	mHydroTurbineGovernor->setParameters(OmRef, R, T1, T2, T3, Pmax, Pmin);
	mHydroTurbineGovernor->initialize(Pref);
	mHasGovernor = true;
	if (mHasTurbine)
	mHasTurbineGovernor=true;
}
//Create a Hydro turbine governor via exsiting object
template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addHydroTurbineGovernor(std::shared_ptr<Signal::HydroTurbineGovernor> hydroTurbineGovernor)
{
	mHydro=true;
	mHydroTurbineGovernor = hydroTurbineGovernor;
	mHasGovernor = true;
	if (mHasTurbine)
	mHasTurbineGovernor=true;
>>>>>>> 8e9cbf324 (HiWi added new Hydro and Steam Turbines and Governor models)
>>>>>>> 338446cac (added new Hydro and Steam Turbines and Governor models)
}
*/

// Declare specializations to move definitions to .cpp
template class CPS::Base::ReducedOrderSynchronGenerator<Real>;
template class CPS::Base::ReducedOrderSynchronGenerator<Complex>;
