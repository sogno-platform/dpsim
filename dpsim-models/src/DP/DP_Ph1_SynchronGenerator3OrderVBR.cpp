/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator3OrderVBR.h>

using namespace CPS;

DP::Ph1::SynchronGenerator3OrderVBR::SynchronGenerator3OrderVBR
    (const String & uid, const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(uid, name, logLevel),
	mEdq_t(Attribute<Matrix>::create("Edq0_t", mAttributes))  {

	//
	mSGOrder = SGOrder::SG3Order;

	// model variables
	**mEdq_t = Matrix::Zero(2,1);
	mEh_vbr = Matrix::Zero(2,1);
}

DP::Ph1::SynchronGenerator3OrderVBR::SynchronGenerator3OrderVBR
	(const String & name, Logger::Level logLevel)
	: SynchronGenerator3OrderVBR(name, name, logLevel) {
}

void DP::Ph1::SynchronGenerator3OrderVBR::specificInitialization() {

	// initial voltage behind the transient reactance in the dq reference frame
	(**mEdq_t)(0,0) = 0.0;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model specific initialization finished ---",
		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0)
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator3OrderVBR::stepInPerUnit() {

	// update Edq_t
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;
	
	// VBR history voltage
	calculateAuxiliarVariables();
	calculateConductanceMatrix();
	mEh_vbr(0,0) = 0.0;
	mEh_vbr(1,0) = mAq_t * (**mIdq)(0,0) + mBq_t * (**mEdq_t)(1,0) + mDq_t * mEf_prev + mDq_t * (**mEf);

	// convert Edq_t into the abc reference frame
	mEvbr = (mKvbr * mEh_vbr * mBase_V_RMS)(0,0);
}
