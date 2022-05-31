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
	: SimPowerComp<Real>(uid, name, logLevel),
	mMechTorque(Attribute<Real>::create("Tm", mAttributes)),
	mElecTorque(Attribute<Real>::create("Te", mAttributes)),
	mDelta(Attribute<Real>::create("delta", mAttributes)),
	mThetaMech(Attribute<Real>::create("Theta", mAttributes)),
	mOmMech(Attribute<Real>::create("w_r", mAttributes)),
	mEf(Attribute<Real>::create("Ef", mAttributes)),
	mVdq0(Attribute<Matrix>::create("Vdq0", mAttributes)),
	mIdq0(Attribute<Matrix>::create("Idq0", mAttributes)) {

	// declare state variables
	**mVdq0 = Matrix::Zero(3,1);
	**mIdq0 = Matrix::Zero(3,1);

	//
	mSimTime = 0.0;
}

template <>
Base::ReducedOrderSynchronGenerator<Complex>::ReducedOrderSynchronGenerator(
	String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel),
	mMechTorque(Attribute<Real>::create("Tm", mAttributes)),
	mElecTorque(Attribute<Real>::create("Te", mAttributes)),
	mDelta(Attribute<Real>::create("delta", mAttributes)),
	mThetaMech(Attribute<Real>::create("Theta", mAttributes)),
	mOmMech(Attribute<Real>::create("w_r", mAttributes)),
	mEf(Attribute<Real>::create("Ef", mAttributes)),
	mVdq(Attribute<Matrix>::create("Vdq0", mAttributes)),
	mIdq(Attribute<Matrix>::create("Idq0", mAttributes))  {

	mSimTime = 0.0;

	// declare state variables
	///FIXME: The mVdq0 and mVdq member variables are mutually exclusive and carry the same attribute name. Maybe they can be unified?
	**mVdq = Matrix::Zero(2,1);
	**mIdq = Matrix::Zero(2,1);
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
	mBase_I = mNomPower / ((3./2.)* mBase_V);
	mBase_Z = mBase_V_RMS / mBase_I_RMS;
	mBase_OmElec = mNomOmega;
	mBase_OmMech = mBase_OmElec;
	mBase_L = mBase_Z / mBase_OmElec;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::setOperationalParametersPerUnit(Real nomPower,
	Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
	Real Ld_t, Real Td0_t) {

	setBaseParameters(nomPower, nomVolt, nomFreq);

	mLd = Ld;
	mLq = Lq;
	mL0 = L0;
	mLd_t = Ld_t;
	mTd0_t = Td0_t;
	mH = H;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::setOperationalParametersPerUnit(Real nomPower,
	Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
	Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t) {

	setBaseParameters(nomPower, nomVolt, nomFreq);

	mLd = Ld;
	mLq = Lq;
	mL0 = L0;
	mLd_t = Ld_t;
	mLq_t = Lq_t;
	mTd0_t = Td0_t;
	mTq0_t = Tq0_t;
	mH = H;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::setOperationalParametersPerUnit(Real nomPower,
	Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
	Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t,
	Real Ld_s, Real Lq_s, Real Td0_s, Real Tq0_s,
	Real Taa) {

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
}

template <>
void Base::ReducedOrderSynchronGenerator<Real>::scaleInertiaConstant(Real scalingFactor) {
	mH = mH * scalingFactor;
	mSLog->info("Scaling inertia with factor {:e}:\n resulting inertia: {:e}\n", scalingFactor, mH);
}

template <>
void Base::ReducedOrderSynchronGenerator<Complex>::scaleInertiaConstant(Real scalingFactor) {
	mH = mH * scalingFactor;
	mSLog->info("Scaling inertia with factor {:e}:\n resulting inertia: {:e}\n", scalingFactor, mH);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::setInitialValues(
	Complex initComplexElectricalPower, Real initMechanicalPower, Complex initTerminalVoltage) {

	mInitElecPower = initComplexElectricalPower;
	mInitMechPower = initMechanicalPower;

	mInitVoltage = initTerminalVoltage;
	mInitVoltageAngle = Math::phase(mInitVoltage);

	mInitCurrent = std::conj(mInitElecPower / mInitVoltage);
	mInitCurrentAngle = Math::phase(mInitCurrent);

	mInitVoltage = mInitVoltage / mBase_V_RMS;
	mInitCurrent = mInitCurrent / mBase_I_RMS;

	mInitialValuesSet = true;
}

template <>
void Base::ReducedOrderSynchronGenerator<Real>::initializeFromNodesAndTerminals(Real frequency) {

	this->updateMatrixNodeIndices();

	if(!mInitialValuesSet)
		this->setInitialValues(-this->terminal(0)->singlePower(), -this->terminal(0)->singlePower().real(), this->initialSingleVoltage(0));

	// Initialize mechanical torque
	**mMechTorque = mInitMechPower / mNomPower;
	mMechTorque_prev = **mMechTorque;
	
	// calculate steady state machine emf (i.e. voltage behind synchronous reactance)
	Complex Eq0 = mInitVoltage + Complex(0, mLq) * mInitCurrent;

	// Load angle
	**mDelta = Math::phase(Eq0);

	// convert currrents to dq reference frame
	(**mIdq0)(0,0) = Math::abs(mInitCurrent) * sin(**mDelta - mInitCurrentAngle);
	(**mIdq0)(1,0) = Math::abs(mInitCurrent) * cos(**mDelta - mInitCurrentAngle);

	// convert voltages to dq reference frame
	(**mVdq0)(0,0) = Math::abs(mInitVoltage) * sin(**mDelta - mInitVoltageAngle);
	(**mVdq0)(1,0) = Math::abs(mInitVoltage) * cos(**mDelta - mInitVoltageAngle);

	// calculate Ef
	**mEf = Math::abs(Eq0) + (mLd - mLq) * (**mIdq0)(0,0);
	mEf_prev = **mEf;

	// Initialize controllers
	if (mHasExciter){
		mExciter->initialize(Math::abs(mInitVoltage), **mEf);
	}
	if (mHasTurbineGovernor){
		mTurbineGovernor->initialize(**mMechTorque);
	}

	// initial electrical torque
	**mElecTorque = (**mVdq0)(0,0) * (**mIdq0)(0,0) + (**mVdq0)(1,0) * (**mIdq0)(1,0);

	// Initialize omega mech with nominal system frequency
	**mOmMech = mNomOmega / mBase_OmMech;

	// initialize theta and calculate transform matrix
	**mThetaMech = **mDelta - PI / 2.;

	mSLog->info(
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

		(**mVdq0)(0,0),
		(**mVdq0)(1,0),
		(**mIdq0)(0,0),
		(**mIdq0)(1,0),
		**mEf,
		**mMechTorque,
		**mElecTorque,
		**mThetaMech,
        **mDelta,
        **mDelta * 180 / PI
	);
	mSLog->flush();
}

template <>
void Base::ReducedOrderSynchronGenerator<Complex>::initializeFromNodesAndTerminals(Real frequency) {

	this->updateMatrixNodeIndices();

	if(!mInitialValuesSet)
		this->setInitialValues(-this->terminal(0)->singlePower(), -this->terminal(0)->singlePower().real(), this->initialSingleVoltage(0));

	// Initialize mechanical torque
	**mMechTorque = mInitMechPower / mNomPower;
	mMechTorque_prev = **mMechTorque;
		
	// calculate steady state machine emf (i.e. voltage behind synchronous reactance)
	Complex Eq0 = mInitVoltage + Complex(0, mLq) * mInitCurrent;

	// Load angle
	**mDelta = Math::phase(Eq0);

	// convert currrents to dq reference frame
	(**mIdq)(0,0) = Math::abs(mInitCurrent) * sin(**mDelta - mInitCurrentAngle);
	(**mIdq)(1,0) = Math::abs(mInitCurrent) * cos(**mDelta - mInitCurrentAngle);

	// convert voltages to dq reference frame
	(**mVdq)(0,0) = Math::abs(mInitVoltage) * sin(**mDelta - mInitVoltageAngle);
	(**mVdq)(1,0) = Math::abs(mInitVoltage) * cos(**mDelta - mInitVoltageAngle);

	// calculate Ef
	**mEf = Math::abs(Eq0) + (mLd - mLq) * (**mIdq)(0,0);
	mEf_prev = **mEf;

	// Initialize controllers
	if (mHasExciter){
		mExciter->initialize(Math::abs(mInitVoltage), **mEf);
	}
	if (mHasTurbineGovernor){
		mTurbineGovernor->initialize(**mMechTorque);
	}

	// initial electrical torque
	**mElecTorque = (**mVdq)(0,0) * (**mIdq)(0,0) + (**mVdq)(1,0) * (**mIdq)(1,0);

	// Initialize omega mech with nominal system frequency
	**mOmMech = mNomOmega / mBase_OmMech;

	// initialize theta and calculate transform matrix
	**mThetaMech = **mDelta - PI / 2.;

	mSLog->info(
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

		(**mVdq)(0,0),
		(**mVdq)(1,0),
		(**mIdq)(0,0),
		(**mIdq)(1,0),
		**mEf,
		**mMechTorque,
		**mElecTorque,
		**mThetaMech,
        **mDelta,
		**mDelta * 180 / PI
	);
	mSLog->flush();
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::mnaInitialize(Real omega,
		Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	MNAInterface::mnaInitialize(omega, timeStep);
	this->updateMatrixNodeIndices();
	mTimeStep = timeStep;
    specificInitialization();

	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

template <>
void Base::ReducedOrderSynchronGenerator<Complex>::MnaPreStep::execute(Real time, Int timeStepCount) {
	mSynGen.mSimTime = time;

	// update controller variables
	if (mSynGen.mHasExciter) {
		mSynGen.mEf_prev = **(mSynGen.mEf);
		**(mSynGen.mEf) = mSynGen.mExciter->step((**mSynGen.mVdq)(0,0), (**mSynGen.mVdq)(1,0), mSynGen.mTimeStep);
	}
	if (mSynGen.mHasTurbineGovernor) {
		mSynGen.mMechTorque_prev = **mSynGen.mMechTorque;
		**mSynGen.mMechTorque = mSynGen.mTurbineGovernor->step(**mSynGen.mOmMech, mSynGen.mTimeStep);
	}

	// calculate mechanical variables at t=k+1 with forward euler
	if (mSynGen.mSimTime>0.0) {
		**mSynGen.mElecTorque = ((**mSynGen.mVdq)(0,0) * (**mSynGen.mIdq)(0,0) + (**mSynGen.mVdq)(1,0) * (**mSynGen.mIdq)(1,0));
		**mSynGen.mOmMech = **mSynGen.mOmMech + mSynGen.mTimeStep * (1. / (2. * mSynGen.mH) * (mSynGen.mMechTorque_prev - **mSynGen.mElecTorque));
		**mSynGen.mThetaMech = **mSynGen.mThetaMech + mSynGen.mTimeStep * (**mSynGen.mOmMech * mSynGen.mBase_OmMech);
		**mSynGen.mDelta = **mSynGen.mDelta + mSynGen.mTimeStep * (**mSynGen.mOmMech - 1.) * mSynGen.mBase_OmMech;
	}

	mSynGen.stepInPerUnit();
	(**mSynGen.mRightVector).setZero();
	mSynGen.mnaApplyRightSideVectorStamp(**mSynGen.mRightVector);
}

template <>
void Base::ReducedOrderSynchronGenerator<Real>::MnaPreStep::execute(Real time, Int timeStepCount) {
	mSynGen.mSimTime = time;
	if (mSynGen.mHasExciter) {
		mSynGen.mEf_prev = **mSynGen.mEf;
		**mSynGen.mEf = mSynGen.mExciter->step((**mSynGen.mVdq0)(0,0), (**mSynGen.mVdq0)(1,0), mSynGen.mTimeStep);
	}
	if (mSynGen.mHasTurbineGovernor) {
		mSynGen.mMechTorque_prev = **mSynGen.mMechTorque;
		**mSynGen.mMechTorque = mSynGen.mTurbineGovernor->step(**mSynGen.mOmMech, mSynGen.mTimeStep);
	}
	
	// calculate mechanical variables at t=k+1 with forward euler
	if (mSynGen.mSimTime>0.0) {
		**mSynGen.mElecTorque = ((**mSynGen.mVdq0)(0,0) * (**mSynGen.mIdq0)(0,0) + (**mSynGen.mVdq0)(1,0) * (**mSynGen.mIdq0)(1,0));
		**mSynGen.mOmMech = **mSynGen.mOmMech + mSynGen.mTimeStep * (1. / (2. * mSynGen.mH) * (mSynGen.mMechTorque_prev - **mSynGen.mElecTorque));
		**mSynGen.mThetaMech = **mSynGen.mThetaMech + mSynGen.mTimeStep * (**mSynGen.mOmMech * mSynGen.mBase_OmMech);
		**mSynGen.mDelta = **mSynGen.mDelta + mSynGen.mTimeStep * (**mSynGen.mOmMech - 1.) * mSynGen.mBase_OmMech;
	}

	mSynGen.stepInPerUnit();
	(**mSynGen.mRightVector).setZero();
	mSynGen.mnaApplyRightSideVectorStamp(**mSynGen.mRightVector);
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSynGen.mnaPostStep(**mLeftVector);
}


template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addExciter(Real Ta, Real Ka, Real Te, Real Ke, 
	Real Tf, Real Kf, Real Tr)
{
	mExciter = Signal::Exciter::make(**this->mName + "_Exciter", this->mLogLevel);
	mExciter->setParameters(Ta, Ka, Te, Ke, Tf, Kf, Tr);
	mHasExciter = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addExciter(
	std::shared_ptr<Signal::Exciter> exciter)
{
	mExciter = exciter;
	mHasExciter = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addGovernor(Real T3, Real T4, Real T5, Real Tc, 
	Real Ts, Real R, Real Pmin, Real Pmax, Real OmRef, Real TmRef)
{
	mTurbineGovernor = Signal::TurbineGovernorType1::make(**this->mName + "_TurbineGovernor", this->mLogLevel);
	mTurbineGovernor->setParameters(T3, T4, T5, Tc, Ts, R, Pmin, Pmax, OmRef);
	mTurbineGovernor->initialize(TmRef);
	mHasTurbineGovernor = true;
}

template <typename VarType>
void Base::ReducedOrderSynchronGenerator<VarType>::addGovernor(
	std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernor)
{
	mTurbineGovernor = turbineGovernor;
	mHasTurbineGovernor = true;
}

// Declare specializations to move definitions to .cpp
template class CPS::Base::ReducedOrderSynchronGenerator<Real>;
template class CPS::Base::ReducedOrderSynchronGenerator<Complex>;
