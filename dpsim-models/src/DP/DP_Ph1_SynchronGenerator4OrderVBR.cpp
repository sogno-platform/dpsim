/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator4OrderVBR.h>

using namespace CPS;

DP::Ph1::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
    (const String & uid, const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(uid, name, logLevel),
	mEdq_t(mAttributes->create<Matrix>("Edq0_t")) {

	//
	mSGOrder = SGOrder::SG4Order;

	// model variables
	**mEdq_t = Matrix::Zero(2,1);
	mEh_vbr = Matrix::Zero(2,1);
}

DP::Ph1::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
	(const String & name, Logger::Level logLevel)
	: SynchronGenerator4OrderVBR(name, name, logLevel) {
}

void DP::Ph1::SynchronGenerator4OrderVBR::specificInitialization() {

	// initial voltage behind the transient reactance in the dq reference frame
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * **mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * **mLd_t;

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Model specific initialization  ---"
		"\nSG model: 4th order"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model specific initialization finished ---",
		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0)
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator4OrderVBR::stepInPerUnit() {

	// update DP-DQ transforms
	mDomainInterface.updateDQToDPTransform(**mThetaMech, mSimTime);
	mDomainInterface.updateDPToDQTransform(**mThetaMech, mSimTime);

	// calculate Edq_t at t=k
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * **mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * **mLd_t;

	// Update time-varying reactance matrix
	calculateConductanceMatrix();

	// VBR history voltage
	mEh_vbr(0,0) = mAd_t * (**mIdq)(1,0) + mBd_t * (**mEdq_t)(0,0);
	mEh_vbr(1,0) = mAq_t * (**mIdq)(0,0) + mBq_t * (**mEdq_t)(1,0) + mDq_t * mEf_prev + mDq_t * (**mEf);

	// convert Edq_t into the abc reference frame
	mEvbr = mDomainInterface.applyDQToDPTransform(mEh_vbr) * mBase_V_RMS;
}
