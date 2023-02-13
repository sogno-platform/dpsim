/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator5bOrderVBR.h>

using namespace CPS;

DP::Ph1::SynchronGenerator5bOrderVBR::SynchronGenerator5bOrderVBR
    (const String & uid, const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(uid, name, logLevel),
	mEdq_t(Attribute<Matrix>::create("Edq_t", mAttributes)),
	mEdq_s(Attribute<Matrix>::create("Edq_s", mAttributes))  {

	//
	mSGOrder = SGOrder::SG5bOrder;

	// model sPecific variables
	**mEdq_t = Matrix::Zero(2,1);
	**mEdq_s = Matrix::Zero(2,1);
	mEh_s = Matrix::Zero(2,1);
	mEh_t = Matrix::Zero(2,1);
}

DP::Ph1::SynchronGenerator5bOrderVBR::SynchronGenerator5bOrderVBR
	(const String & name, Logger::Level logLevel)
	: SynchronGenerator5bOrderVBR(name, name, logLevel) {
}

void DP::Ph1::SynchronGenerator5bOrderVBR::specificInitialization() {

	// initial voltage behind the transient reactance in the dq reference frame
	(**mEdq_t)(0,0) = 0.0;
	(**mEdq_t)(1,0) = (1 - mTaa / mTd0_t) * **mEf - (mLd - mLd_t - mYd) * (**mIdq)(0,0);

	// initial dq behind the subtransient reactance in the dq reference frame
	(**mEdq_s)(0,0) = (**mVdq)(0,0) - (mLq_s) * (**mIdq)(1,0);
	(**mEdq_s)(1,0) = (**mVdq)(1,0) + (mLd_s) * (**mIdq)(0,0);

	mSLog->info(
		"\n--- Model ssecific initialization  ---"
		"\nSG model: 5th order type 2"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nInitial Ed_s (per unit): {:f}"
		"\nInitial Eq_s (per unit): {:f}"
		"\n--- Model specific initialization finished ---",
		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0),
		(**mEdq_s)(0,0),
		(**mEdq_s)(1,0)
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator5bOrderVBR::stepInPerUnit() {
	if (mSimTime>0.0){
		// calculate Edq_t at t=k
		(**mEdq_t)(0,0) = 0.0;
		(**mEdq_t)(1,0) = mAq_t * (**mIdq)(0,0) + mEh_t(1,0);

		// calculate Edq_s at t=k
		(**mEdq_s)(0,0) = (**mVdq)(0,0) - mLq_s * (**mIdq)(1,0);
		(**mEdq_s)(1,0) = (**mVdq)(1,0) + mLd_s * (**mIdq)(0,0);
	}

	// VBR history voltage
	calculateAuxiliarVariables();
	calculateConductanceMatrix();

	// calculate history term behind the transient reactance
	mEh_t(0,0) = 0.0;
	mEh_t(1,0) = mAq_t * (**mIdq)(0,0) + mBq_t * (**mEdq_t)(1,0) + mDq_t * (**mEf) + mDq_t * mEf_prev;

	// calculate history term behind the subtransient reactance
	mEh_s(0,0) = mAd_s * (**mIdq)(1,0) + mCd_s * (**mEdq_s)(0,0);
	mEh_s(1,0) = mAq_s * (**mIdq)(0,0) + mBq_s * (**mEdq_t)(1,0) + mCq_s * (**mEdq_s)(1,0) + mDq_s * (**mEf) + mDq_s * mEf_prev;
	
	// convert Edq_t into the abc reference frame
	mEvbr = (mKvbr * mEh_s * mBase_V_RMS)(0,0);
}