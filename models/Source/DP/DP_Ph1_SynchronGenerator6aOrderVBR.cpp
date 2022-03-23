/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_SynchronGenerator6aOrderVBR.h>

using namespace CPS;

DP::Ph1::SynchronGenerator6aOrderVBR::SynchronGenerator6aOrderVBR
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

DP::Ph1::SynchronGenerator6aOrderVBR::SynchronGenerator6aOrderVBR
	(String name, Logger::Level logLevel)
	: SynchronGenerator6aOrderVBR(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator6aOrderVBR::clone(String name) {
	auto copy = SynchronGenerator6aOrderVBR::make(name, mLogLevel);
	
	return copy;
}

void DP::Ph1::SynchronGenerator6aOrderVBR::specificInitialization() {

	// calculate auxiliar constants
	calculateAuxiliarConstants();

	// initial voltage behind the transient reactance in the dq reference frame
	mEdq_t(0,0) = (mLq - mLq_t - mYq) * mIdq(1,0);
	mEdq_t(1,0) = (1 - mTaa / mTd0_t) * mEf - (mLd - mLd_t - mYd) * mIdq(0,0);

	// initial dq behind the subtransient reactance in the dq reference frame
	mEdq_s(0,0) = mVdq(0,0) - mLq_s * mIdq(1,0);
	mEdq_s(1,0) = mVdq(1,0) + mLd_s * mIdq(0,0);

	// constant part of ABC resistance matrix
	mResistanceMatrix_const = Matrix::Zero(1,3);
	mResistanceMatrix_const <<	-mL0,	-sqrt(3) / 2. * (mA - mB) - mL0,	sqrt(3) / 2. * (mA - mB) - mL0;
	mResistanceMatrix_const = (-1. / 3.) * mResistanceMatrix_const;
	mR_const_1ph = (mResistanceMatrix_const * mShiftVector)(0,0);

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

void DP::Ph1::SynchronGenerator6aOrderVBR::calculateAuxiliarConstants() {
	mYd = (mTd0_s / mTd0_t) * (mLd_s / mLd_t) * (mLd - mLd_t);
	mYq = (mTq0_s / mTq0_t) * (mLq_s / mLq_t) * (mLq - mLq_t);
	Real Zq_t = mLd - mLd_t - mYd;
	Real Zd_t = mLq - mLq_t - mYq;
	Real Zq_s = mLd_t - mLd_s + mYd;
	Real Zd_s = mLq_t - mLq_s + mYq;
	Real Tf = mTaa / mTd0_t;

	mAd_t = mTimeStep * Zd_t / (2 * mTq0_t + mTimeStep);
	mBd_t = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);
	mAq_t = - mTimeStep * Zq_t / (2 * mTd0_t + mTimeStep);
	mBq_t = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
	mDq_t = 2 * mTimeStep * (1 - Tf) / (2 * mTd0_t + mTimeStep);

	mAd_s = (mTimeStep * Zd_s + mTimeStep * mAd_t) / (2 * mTq0_s + mTimeStep);
	mBd_s = (mTimeStep * mBd_t + mTimeStep) / (2 * mTq0_s + mTimeStep);
	mCd_s = (2 * mTq0_s - mTimeStep) / (2 * mTq0_s + mTimeStep);
	mAq_s = (-mTimeStep * Zq_s + mTimeStep * mAq_t ) / (2 * mTd0_s + mTimeStep);
	mBq_s = (mTimeStep * mBq_t + mTimeStep) / (2 * mTd0_s + mTimeStep);
	mCq_s = (2 * mTd0_s - mTimeStep) / (2 * mTd0_s + mTimeStep);
	mDq_s = (mTimeStep * mDq_t + 2 * Tf * mTimeStep) / (2 * mTd0_s + mTimeStep);

	mB = mLd_s - mAq_s;
	mA = -mLq_s - mAd_s;

	mKc = Matrix::Zero(1,3);
	mKc << Complex(cos(PI/2.), -sin(PI/2.)), Complex(cos(7.*PI/6.), -sin(7.*PI/6.)),	Complex(cos(PI/6.), sin(PI/6.));
	mKc = (-1. / 6.) * (mA + mB) * mKc;
}

void DP::Ph1::SynchronGenerator6aOrderVBR::stepInPerUnit() {

	if (mSimTime>0.0){
		// calculate Edq_t at t=k
		mEdq_t(0,0) = mAd_t * mIdq(1,0) + mEh_t(0,0);
		mEdq_t(1,0) = mAq_t * mIdq(0,0) + mEh_t(1,0);

		// calculate Edq_s at t=k
		mEdq_s(0,0) = -mIdq(1,0) * mLq_s + mVdq(0,0);
		mEdq_s(1,0) = mIdq(0,0) * mLd_s + mVdq(1,0);

		mElecTorque = mVdq(0,0) * mIdq(0,0) + mVdq(1,0) * mIdq(1,0);
		mOmMech = mOmMech + mTimeStep * (1. / (2. * mH) * (mMechTorque - mElecTorque));
		mThetaMech = mThetaMech + mTimeStep * (mOmMech * mBase_OmMech);
		mDelta = mDelta + mTimeStep * (mOmMech - 1.) * mBase_OmMech;
	}

	// VBR history voltage
	calculateAuxiliarVariables();
	calculateConductanceMatrix();
	
	// calculate history term behind the transient reactance
	mEh_t(0,0) = mAd_t * mIdq(1,0) + mBd_t * mEdq_t(0,0);
	mEh_t(1,0) = mAq_t * mIdq(0,0) + mBq_t * mEdq_t(1,0) + mDq_t * mEf;

	// calculate history term behind the subtransient reactance
	mEh_s(0,0) = mAd_s * mIdq(1,0) + mBd_s * mEdq_t(0,0) + mCd_s * mEdq_s(0,0);
	mEh_s(1,0) = mAq_s * mIdq(0,0) + mBq_s * mEdq_t(1,0) + mCq_s * mEdq_s(1,0) + mDq_s * mEf;

	// convert Edq_t into the abc reference frame
	mEvbr = (mKvbr * mEh_s * mBase_V_RMS)(0,0);
}
