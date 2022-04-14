/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_SynchronGenerator4OrderVBR.h>

using namespace CPS;

SP::Ph1::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
    (String uid, String name, Logger::Level logLevel)
	: SynchronGeneratorVBR(uid, name, logLevel) {

	// model specific variables
	mEdq_t = Matrix::Zero(2,1);
	mEh_vbr = Matrix::Zero(2,1);

    // Register attributes
	addAttribute<Matrix>("Edq_t", &mEdq_t, Flags::read);
}

SP::Ph1::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
	(String name, Logger::Level logLevel)
	: SynchronGenerator4OrderVBR(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr SP::Ph1::SynchronGenerator4OrderVBR::clone(String name) {
	auto copy = SynchronGenerator4OrderVBR::make(name, mLogLevel);
	return copy;
}

void SP::Ph1::SynchronGenerator4OrderVBR::setOperationalParametersPerUnit(Real nomPower, 
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t) {

	Base::ReducedOrderSynchronGenerator<Complex>::setOperationalParametersPerUnit(nomPower, 
			nomVolt, nomFreq, H, Ld, Lq, L0,
			Ld_t, Lq_t, Td0_t, Tq0_t);
	
	mSLog->info("Set base parameters: \n"
				"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n",
				nomPower, nomVolt, nomFreq);

	mSLog->info("Set operational parameters in per unit: \n"
			"inertia: {:e}\n"
			"Ld: {:e}\nLq: {:e}\nL0: {:e}\n"
			"Ld_t: {:e}\nLq_t: {:e}\n"
			"Td0_t: {:e}\nTq0_t: {:e}\n",
			H, Ld, Lq, L0, 
			Ld_t, Lq_t,
			Td0_t, Tq0_t);
};


void SP::Ph1::SynchronGenerator4OrderVBR::specificInitialization() {

	// initial voltage behind the transient reactance in the dq reference frame
	mEdq_t(0,0) = mVdq(0,0) - mIdq(1,0) * mLq_t;
	mEdq_t(1,0) = mVdq(1,0) + mIdq(0,0) * mLd_t;

	// initialize conductance matrix 
	mConductanceMatrix = Matrix::Zero(2,2);

	// auxiliar VBR constants
	calculateAuxiliarConstants();

	// calculate resistance matrix in dq reference frame
	mResistanceMatrixDq = Matrix::Zero(2,2);
	mResistanceMatrixDq <<	0.0,			-mAd -mLq_t,
					  		mLd_t - mAq,	0.0;

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model specific initialization finished ---",

		mEdq_t(0,0),
		mEdq_t(1,0)
	);
	mSLog->flush();
}

void SP::Ph1::SynchronGenerator4OrderVBR::calculateAuxiliarConstants() {
	mAd = mTimeStep * (mLq - mLq_t) / (2 * mTq0_t + mTimeStep);
	mBd = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);

	mAq = - mTimeStep * (mLd - mLd_t) / (2 * mTd0_t + mTimeStep);
	mBq = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
	mCq = 2 * mTimeStep * mEf / (2 * mTd0_t + mTimeStep);
}

void SP::Ph1::SynchronGenerator4OrderVBR::stepInPerUnit() {
	
	if (mSimTime>0.0) {
		// calculate Edq_t at t=k
		mEdq_t(0,0) = -mIdq(1,0) * mLq_t + mVdq(0,0);
		mEdq_t(1,0) = mIdq(0,0) * mLd_t + mVdq(1,0);

		// calculate mechanical variables at t=k+1 with forward euler
		mElecTorque = (mVdq(0,0) * mIdq(0,0) + mVdq(1,0) * mIdq(1,0));
		mOmMech = mOmMech + mTimeStep * (1. / (2. * mH) * (mMechTorque - mElecTorque));
		mThetaMech = mThetaMech + mTimeStep * (mOmMech * mBase_OmMech);
		mDelta = mDelta + mTimeStep * (mOmMech - 1.) * mBase_OmMech;
	}

	// get transformation matrix
	mDqToComplexA = get_DqToComplexATransformMatrix();
	mComplexAToDq = mDqToComplexA.transpose();

	// calculate resistance matrix at t=k+1
	calculateResistanceMatrix();

	// VBR history voltage
	mEh_vbr(0,0) = mAd * mIdq(1,0) + mBd * mEdq_t(0,0);
	mEh_vbr(1,0) = mAq * mIdq(0,0) + mBq * mEdq_t(1,0) + mCq;
	
	// convert Edq_t into the abc reference frame
	mEh_vbr = mDqToComplexA * mEh_vbr;
	Evbr = Complex(mEh_vbr(0,0), mEh_vbr(1,0)) * mBase_V_RMS;
}