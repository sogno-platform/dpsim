/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SynchronGenerator6bOrderVBR.h>

using namespace CPS;

EMT::Ph3::SynchronGenerator6bOrderVBR::SynchronGenerator6bOrderVBR
    (String uid, String name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(uid, name, logLevel) {

	// model specific variables
	mEdq0_t = Matrix::Zero(3,1);
	mEdq0_s = Matrix::Zero(3,1);
	mEh_t = Matrix::Zero(3,1);
	mEh_s = Matrix::Zero(3,1);

    // Register attributes
	addAttribute<Matrix>("Edq0_t", &mEdq0_t, Flags::read);
	addAttribute<Matrix>("Edq0_s", &mEdq0_s, Flags::read);
}

EMT::Ph3::SynchronGenerator6bOrderVBR::SynchronGenerator6bOrderVBR
	(String name, Logger::Level logLevel)
	: SynchronGenerator6bOrderVBR(name, name, logLevel) {
}

SimPowerComp<Real>::Ptr EMT::Ph3::SynchronGenerator6bOrderVBR::clone(String name) {
	
	auto copy = SynchronGenerator6bOrderVBR::make(name, mLogLevel);
	return copy;
}

void EMT::Ph3::SynchronGenerator6bOrderVBR::setOperationalParametersPerUnit(Real nomPower, 
		Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
		Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t,
		Real Ld_s, Real Lq_s, Real Td0_s, Real Tq0_s) {

	Base::ReducedOrderSynchronGenerator<Real>::setOperationalParametersPerUnit(nomPower, 
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

void EMT::Ph3::SynchronGenerator6bOrderVBR::specificInitialization() {
	// initial voltage behind the transient reactance in the dq reference frame
	mEdq0_t(0,0) = (mLq - mLq_t) * mIdq0(1,0);
	mEdq0_t(1,0) = mEf - (mLd - mLd_t) * mIdq0(0,0);

	// initial dq behind the subtransient reactance in the dq reference frame
	mEdq0_s(0,0) = mVdq0(0,0) - mLq_s * mIdq0(1,0);
	mEdq0_s(1,0) = mVdq0(1,0) + mLd_s * mIdq0(0,0);

	// calculate auxiliar VBR constants
	calculateAuxiliarConstants();

	// dq0 resistance matrix
	mResistanceMatrixDq0 = Matrix::Zero(3,3);
	mResistanceMatrixDq0 <<	0.0,			-mAd_s -mLq_s,	0.0,
							mLd_s - mAq_s,	0.0,			0.0,
					  		0.0,			0.0,			mL0;

	// initialize conductance matrix 
	mConductanceMatrix = Matrix::Zero(3,3);

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nInitial Ed_s (per unit): {:f}"
		"\nInitial Eq_s (per unit): {:f}"
		"\n--- Model specific initialization finished ---",

		mEdq0_t(0,0),
		mEdq0_t(1,0),
		mEdq0_s(0,0),
		mEdq0_s(1,0)
	);
	mSLog->flush();
}

void EMT::Ph3::SynchronGenerator6bOrderVBR::calculateAuxiliarConstants() {
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

void EMT::Ph3::SynchronGenerator6bOrderVBR::stepInPerUnit() {

	if (mSimTime>0.0) {
		// calculate Edq_t at t=k
		mEdq0_t(0,0) = mAd_t * mIdq0(1,0) + mEh_t(0,0);
		mEdq0_t(1,0) = mAq_t * mIdq0(0,0) + mEh_t(1,0);
		mEdq0_t(2,0) = 0.0;

		// calculate Edq_s at t=k
		mEdq0_s(0,0) = -mIdq0(1,0) * mLq_s + mVdq0(0,0);
		mEdq0_s(1,0) = mIdq0(0,0) * mLd_s + mVdq0(1,0);

		// calculate mechanical variables at t=k+1 with forward euler
		mElecTorque = (mVdq0(0,0) * mIdq0(0,0) + mVdq0(1,0) * mIdq0(1,0));
		mOmMech = mOmMech + mTimeStep * (1. / (2. * mH) * (mMechTorque - mElecTorque));
		mThetaMech = mThetaMech + mTimeStep * (mOmMech * mBase_OmMech);
		mDelta = mDelta + mTimeStep * (mOmMech - 1.) * mBase_OmMech;
	}

	// get transformation matrix
	mAbcToDq0 = get_parkTransformMatrix();
	mDq0ToAbc = get_inverseParkTransformMatrix();

	// calculate resistance matrix at t=k+1
	calculateResistanceMatrix();

	// calculate history term behind the transient reactance
	mEh_t(0,0) = mAd_t * mIdq0(1,0) + mBd_t * mEdq0_t(0,0);
	mEh_t(1,0) = mAq_t * mIdq0(0,0) + mBq_t * mEdq0_t(1,0) + mDq_t * mEf;
	mEh_t(2,0) = 0.0;

	// calculate history term behind the subtransient reactance
	mEh_s(0,0) = mAd_s * mIdq0(1,0) + mBd_s * mEdq0_t(0,0) + mCd_s * mEdq0_s(0,0);
	mEh_s(1,0) = mAq_s * mIdq0(0,0) + mBq_s * mEdq0_t(1,0) + mCq_s * mEdq0_s(1,0) + mDq_s * mEf;
	mEh_s(2,0) = 0.0;

	// convert Edq_s into the abc reference frame
	mEvbr = mDq0ToAbc * mEh_s * mBase_V;
}

