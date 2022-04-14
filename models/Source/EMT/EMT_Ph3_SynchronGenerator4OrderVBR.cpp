/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SynchronGenerator4OrderVBR.h>

using namespace CPS;

EMT::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
    (String uid, String name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(uid, name, logLevel) {

	// model specific variables
	mEdq0_t = Matrix::Zero(3,1);
	mEhs_vbr = Matrix::Zero(3,1);

    // Register attributes
	addAttribute<Matrix>("Edq0_t", &mEdq0_t, Flags::read);
}

EMT::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
	(String name, Logger::Level logLevel)
	: SynchronGenerator4OrderVBR(name, name, logLevel) {
}

SimPowerComp<Real>::Ptr EMT::Ph3::SynchronGenerator4OrderVBR::clone(String name) {
	
	auto copy = SynchronGenerator4OrderVBR::make(name, mLogLevel);
	return copy;
}

void EMT::Ph3::SynchronGenerator4OrderVBR::setOperationalParametersPerUnit(Real nomPower, 
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t) {

	Base::ReducedOrderSynchronGenerator<Real>::setOperationalParametersPerUnit(nomPower, 
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

void EMT::Ph3::SynchronGenerator4OrderVBR::specificInitialization() {
	// initial voltage behind the transient reactance in the dq0 reference frame
	mEdq0_t(0,0) = mVdq0(0,0) - mIdq0(1,0) * mLq_t;
	mEdq0_t(1,0) = mVdq0(1,0) + mIdq0(0,0) * mLd_t;

	// calculate auxiliar VBR constants
	calculateAuxiliarConstants();

	// dq0 resistance matrix
	mResistanceMatrixDq0 = Matrix::Zero(3,3);
	mResistanceMatrixDq0 <<	0.0,			-mAd -mLq_t,	0.0,
							mLd_t - mAq,	0.0,			0.0,
					  		0.0,			0.0,			mL0;

	// initialize conductance matrix 
	mConductanceMatrix = Matrix::Zero(3,3);

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model specific initialization finished ---",

		mEdq0_t(0,0),
		mEdq0_t(1,0)
	);
	mSLog->flush();
}

void EMT::Ph3::SynchronGenerator4OrderVBR::calculateAuxiliarConstants() {
	mAd = mTimeStep * (mLq - mLq_t) / (2 * mTq0_t + mTimeStep);
	mBd = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);

	mAq = - mTimeStep * (mLd - mLd_t) / (2 * mTd0_t + mTimeStep);
	mBq = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
	mCq = 2 * mTimeStep * mEf / (2 * mTd0_t + mTimeStep);
}

void EMT::Ph3::SynchronGenerator4OrderVBR::stepInPerUnit() {

	if (mSimTime>0.0) {
		// calculate Edq_t at t=k
		mEdq0_t(0,0) = -mIdq0(1,0) * mLq_t + mVdq0(0,0);
		mEdq0_t(1,0) = mIdq0(0,0) * mLd_t + mVdq0(1,0);
		mEdq0_t(2,0) = 0.0;

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

	// VBR history voltage
	mEhs_vbr(0,0) = mAd * mIdq0(1,0) + mBd * mEdq0_t(0,0);
	mEhs_vbr(1,0) = mAq * mIdq0(0,0) + mBq * mEdq0_t(1,0) + mCq;
	mEhs_vbr(2,0) = 0.0;

	// convert Edq_t into the abc reference frame
	mEvbr = mDq0ToAbc * mEhs_vbr * mBase_V;
}

