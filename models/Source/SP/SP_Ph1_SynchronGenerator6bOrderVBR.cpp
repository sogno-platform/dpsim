/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_SynchronGenerator6bOrderVBR.h>

using namespace CPS;

SP::Ph1::SynchronGenerator6bOrderVBR::SynchronGenerator6bOrderVBR
    (String uid, String name, Logger::Level logLevel)
	: SynchronGeneratorVBR(uid, name, logLevel) {

	// model specific variables
	mEdq_t = Matrix::Zero(2,1);
	mEdq_s = Matrix::Zero(2,1);
	mEh_s = Matrix::Zero(2,1);
	mEh_t = Matrix::Zero(2,1);

    // Register attributes
	addAttribute<Matrix>("Edq_t", &mEdq_t, Flags::read);
	addAttribute<Matrix>("Edq_s", &mEdq_s, Flags::read);
}

SP::Ph1::SynchronGenerator6bOrderVBR::SynchronGenerator6bOrderVBR
	(String name, Logger::Level logLevel)
	: SynchronGenerator6bOrderVBR(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr SP::Ph1::SynchronGenerator6bOrderVBR::clone(String name) {
	auto copy = SynchronGenerator6bOrderVBR::make(name, mLogLevel);
	return copy;
}

void SP::Ph1::SynchronGenerator6bOrderVBR::setOperationalParametersPerUnit(Real nomPower, 
		Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
		Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t,
		Real Ld_s, Real Lq_s, Real Td0_s, Real Tq0_s) {

	Base::ReducedOrderSynchronGenerator<Complex>::setOperationalParametersPerUnit(nomPower, 
		nomVolt, nomFreq, H, Ld, Lq, L0,
		Ld_t, Lq_t, Td0_t, Tq0_t,
		Ld_s, Lq_s, Td0_s, Tq0_s);
	
	mSLog->info("Set base parameters: \n"
				"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n",
				nomPower, nomVolt, nomFreq);

	mSLog->info("Set operational parameters in per unit: \n"
			"inertia: {:e}\n"
			"Ld: {:e}\nLq: {:e}\nL0: {:e}\n"
			"Ld_t: {:e}\nLq_t: {:e}\n"
			"Td0_t: {:e}\nTq0_t: {:e}\n"
			"Ld_s: {:e}\nLq_s: {:e}\n"
			"Td0_s: {:e}\nTq0_s: {:e}\n",
			H, Ld, Lq, L0, 
			Ld_t, Lq_t,
			Td0_t, Tq0_t,
			Ld_s, Lq_s,
			Td0_s, Tq0_s);
};

void SP::Ph1::SynchronGenerator6bOrderVBR::specificInitialization() {

	// initial voltage behind the transient reactance in the dq reference frame
	mEdq_t(0,0) = (mLq - mLq_t) * mIdq(1,0);
	mEdq_t(1,0) = mEf - (mLd - mLd_t) * mIdq(0,0);

	// initial dq behind the subtransient reactance in the dq reference frame
	mEdq_s(0,0) = mVdq(0,0) - mLq_s * mIdq(1,0);
	mEdq_s(1,0) = mVdq(1,0) + mLd_s * mIdq(0,0);

	// initialize conductance matrix 
	mConductanceMatrix = Matrix::Zero(2,2);

	// auxiliar VBR constants
	calculateAuxiliarConstants();

	// calculate resistance matrix in dq reference frame
	mResistanceMatrixDq = Matrix::Zero(2,2);
	mResistanceMatrixDq <<	0.0,			-mLq_s - mAd_s,
					  		mLd_s - mAq_s,	0.0;

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nInitial Ed_s (per unit): {:f}"
		"\nInitial Eq_s (per unit): {:f}"
		"\n--- Model specific initialization finished ---",

		mEdq_t(0,0),
		mEdq_t(1,0),
		mEdq_s(0,0),
		mEdq_s(1,0)
	);
	mSLog->flush();
}

void SP::Ph1::SynchronGenerator6bOrderVBR::calculateAuxiliarConstants() {
	mAd_t = mTimeStep * (mLq - mLq_t) / (2 * mTq0_t + mTimeStep);
	mBd_t = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);
	mAq_t = - mTimeStep * (mLd - mLd_t) / (2 * mTd0_t + mTimeStep);
	mBq_t = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
	mDq_t = 2 * mTimeStep / (2 * mTd0_t + mTimeStep);

	mAd_s = (mTimeStep * (mLq_t - mLq_s) + mTimeStep * mAd_t) / (2 * mTq0_s + mTimeStep);
	mBd_s = (mTimeStep * mBd_t + mTimeStep) / (2 * mTq0_s + mTimeStep);
	mCd_s = (2 * mTq0_s - mTimeStep) / (2 * mTq0_s + mTimeStep);
	mAq_s = (-mTimeStep * (mLd_t - mLd_s) + mTimeStep * mAq_t ) / (2 * mTd0_s + mTimeStep);
	mBq_s = (mTimeStep * mBq_t + mTimeStep) / (2 * mTd0_s + mTimeStep);
	mCq_s = (2 * mTd0_s - mTimeStep) / (2 * mTd0_s + mTimeStep);
	mDq_s = mTimeStep * mDq_t / (2 * mTd0_s + mTimeStep);
}

void SP::Ph1::SynchronGenerator6bOrderVBR::stepInPerUnit() {
	if (mSimTime>0.0) {
		// calculate Edq_t at t=k
		mEdq_t(0,0) = mAd_t * mIdq(1,0) + mEh_t(0,0);
		mEdq_t(1,0) = mAq_t * mIdq(0,0) + mEh_t(1,0);

		// calculate Edq_s at t=k
		mEdq_s(0,0) = -mIdq(1,0) * mLq_s + mVdq(0,0);
		mEdq_s(1,0) = mIdq(0,0) * mLd_s + mVdq(1,0);

		// calculate mechanical variables at t=k+1 with forward euler
		mElecTorque = (mVdq(0,0) * mIdq(0,0) + mVdq(1,0) * mIdq(1,0));
		mOmMech = mOmMech + mTimeStep * (1. / (2. * mH) * (mMechTorque - mElecTorque));
		mThetaMech = mThetaMech + mTimeStep * (mOmMech * mBase_OmMech);
		mDelta = mDelta + mTimeStep * (mOmMech - 1.) * mBase_OmMech;
	}

	mDqToComplexA = get_DqToComplexATransformMatrix();
	mComplexAToDq = mDqToComplexA.transpose();

	// calculate resistance matrix at t=k+1
	calculateResistanceMatrix();

	// calculate history term behind the transient reactance
	mEh_t(0,0) = mAd_t * mIdq(1,0) + mBd_t * mEdq_t(0,0);
	mEh_t(1,0) = mAq_t * mIdq(0,0) + mBq_t * mEdq_t(1,0) + mDq_t * mEf;

	// calculate history term behind the subtransient reactance
	mEh_s(0,0) = mAd_s * mIdq(1,0) + mBd_s * mEdq_t(0,0) + mCd_s * mEdq_s(0,0);
	mEh_s(1,0) = mAq_s * mIdq(0,0) + mBq_s * mEdq_t(1,0) + mCq_s * mEdq_s(1,0) + mDq_s * mEf;
	
	// convert Edq_t into the abc reference frame
	mEh_s = mDqToComplexA * mEh_s;
	Evbr = Complex(mEh_s(0,0), mEh_s(1,0)) * mBase_V_RMS;
}