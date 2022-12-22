/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGenerator4OrderVBR.h>

using namespace CPS;

EMT::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
    (const String & uid, const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(uid, name, logLevel),
	mEdq0_t(mAttributes->create<Matrix>("Edq0_t")) {

	//
	mSGOrder = SGOrder::SG4Order;

	// model specific variables
	**mEdq0_t = Matrix::Zero(3,1);
	mEhs_vbr = Matrix::Zero(3,1);
}

EMT::Ph3::SynchronGenerator4OrderVBR::SynchronGenerator4OrderVBR
	(const String & name, Logger::Level logLevel)
	: SynchronGenerator4OrderVBR(name, name, logLevel) {
}

void EMT::Ph3::SynchronGenerator4OrderVBR::specificInitialization() {
	// initial voltage behind the transient reactance in the dq0 reference frame
	(**mEdq0_t)(0,0) = (**mVdq0)(0,0) - (**mIdq0)(1,0) * mLq_t;
	(**mEdq0_t)(1,0) = (**mVdq0)(1,0) + (**mIdq0)(0,0) * mLd_t;

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model specific initialization finished ---",

		(**mEdq0_t)(0,0),
		(**mEdq0_t)(1,0)
	);
	mSLog->flush();
}

void EMT::Ph3::SynchronGenerator4OrderVBR::stepInPerUnit() {

	if (mSimTime>0.0) {
		// calculate Edq_t at t=k
		(**mEdq0_t)(0,0) = -(**mIdq0)(1,0) * mLq_t + (**mVdq0)(0,0);
		(**mEdq0_t)(1,0) = (**mIdq0)(0,0) * mLd_t + (**mVdq0)(1,0);
		(**mEdq0_t)(2,0) = 0.0;
	}

	// get transformation matrix
	mAbcToDq0 = get_parkTransformMatrix();
	mDq0ToAbc = get_inverseParkTransformMatrix();

	// calculate resistance matrix at t=k+1
	calculateResistanceMatrix();

	// VBR history voltage
	mEhs_vbr(0,0) = mAd_t * (**mIdq0)(1,0) + mBd_t * (**mEdq0_t)(0,0);
	mEhs_vbr(1,0) = mAq_t * (**mIdq0)(0,0) + mBq_t * (**mEdq0_t)(1,0) + mDq_t * mEf_prev + mDq_t * (**mEf);
	mEhs_vbr(2,0) = 0.0;

	// convert Edq_t into the abc reference frame
	mEvbr = mDq0ToAbc * mEhs_vbr * mBase_V;
}


// #### DAE functions ####

void EMT::Ph3::SynchronGenerator4OrderVBR::daeInitialize(double time, double state[], double dstate_dt[], 
	double absoluteTolerances[], double stateVarTypes[], int& offset) {
	// state variables are: Ed_t, Eq_t, omega and mechanical theta
	updateMatrixNodeIndices();

	// initial voltage behind the transient reactance in the dq0 reference frame
	(**mEdq0_t)(0,0) = (**mVdq0)(0,0) - (**mIdq0)(1,0) * mLq_t;
	(**mEdq0_t)(1,0) = (**mVdq0)(1,0) + (**mIdq0)(0,0) * mLd_t;

	// init state variables
	state[offset] = (**mEdq0_t)(0,0);
	dstate_dt[offset] = 0.0;
	state[offset+1] = (**mEdq0_t)(1,0);
	dstate_dt[offset+1] = 0.0;
	state[offset+2] = **mOmMech;
	dstate_dt[offset+2] = 0.0;
	state[offset+3] = **mThetaMech;
	dstate_dt[offset+3] = mBase_OmMech * state[offset+2];
	state[offset+4] = (**mIdq0)(0,0);
	dstate_dt[offset+4] = 0.0;
	state[offset+5] = (**mIdq0)(1,0);
	dstate_dt[offset+5] = 0.0;

	// set state variables as differential variable
	stateVarTypes[offset+0] = 0.0;
	stateVarTypes[offset+1] = 0.0;
	stateVarTypes[offset+2] = 0.0;
	stateVarTypes[offset+3] = 0.0;
	stateVarTypes[offset+4] = 1.0;
	stateVarTypes[offset+5] = 1.0;

	// set absolute tolerance
	absoluteTolerances[offset]   = mAbsTolerance;
	absoluteTolerances[offset+1] = mAbsTolerance;
	absoluteTolerances[offset+2] = mAbsTolerance;
	absoluteTolerances[offset+3] = mAbsTolerance;
	absoluteTolerances[offset+4] = mAbsTolerance;
	absoluteTolerances[offset+5] = mAbsTolerance;

	mSLog->info(
		"\n--- daeInitialize ---"
		"\nInitial Ed_t = {:f} [p.u.]"
		"\nInitial Eq_t = {:f} [p.u.]"
		"\nInitial mechanical omega = {:f} [p.u.]"
		"\nInitial mechanical theta = {:f} [p.u.]"
		"\nInitial derivative of mechanical theta = {:f} [p.u.]"
		"\n--- daeInitialize finished ---",
		state[offset],
		state[offset+1],
		state[offset+2],
		state[offset+3],
		dstate_dt[offset+3]
	);
	mSLog->flush();
	offset+=6;
}

void EMT::Ph3::SynchronGenerator4OrderVBR::daeResidual(double sim_time, 
	const double state[], const double dstate_dt[], 
	double resid[], std::vector<int>& off) {

	// current offset for component	
	int c_offset = off[0] + off[1]; 

	// get transformation matrix
	**mThetaMech = state[c_offset+3];
	mAbcToDq0 = get_parkTransformMatrix();
	mDq0ToAbc = get_inverseParkTransformMatrix();

	// convert terminal voltage into dq0 reference frame
	(**mIntfVoltage)(0, 0) = state[matrixNodeIndex(0, 0)];
	(**mIntfVoltage)(1, 0) = state[matrixNodeIndex(0, 1)];
	(**mIntfVoltage)(2, 0) = state[matrixNodeIndex(0, 2)];
	**mVdq0 = mAbcToDq0 * **mIntfVoltage / mBase_V;

	// calculate dq0 and abc currents
	//(**mEdq0_t)(0,0) = state[c_offset];
	//(**mEdq0_t)(1,0) = state[c_offset+1];
	//(**mIdq0)(0,0) = ((**mEdq0_t)(1,0) - (**mVdq0)(1,0)) / mLd_t;
	//(**mIdq0)(1,0) = ((**mVdq0)(0,0) - (**mEdq0_t)(0,0)) / mLq_t;
	
	
	// residual function for Edq0_t
	//resid[c_offset] = mTq0_t * dstate_dt[c_offset] + state[c_offset] - (mLq - mLq_t) * (**mIdq0)(1,0);
	//resid[c_offset+1] = mTd0_t * dstate_dt[c_offset+1] + state[c_offset+1] - mEf + (mLd - mLd_t) * (**mIdq0)(0,0);
	resid[c_offset] = mTq0_t * dstate_dt[c_offset] + state[c_offset] - (mLq - mLq_t) * state[c_offset+5];
	resid[c_offset+1] = mTd0_t * dstate_dt[c_offset+1] + state[c_offset+1] - mEf + (mLd - mLd_t) * state[c_offset+4];

	// residual function for omega
	(**mIdq0)(0,0) = state[c_offset+4];
	(**mIdq0)(1,0) = state[c_offset+5];
	//**mElecTorque = ((**mVdq0)(0,0) * (**mIdq0)(0,0) + (**mVdq0)(1,0) * (**mIdq0)(1,0));
	resid[c_offset+2] = 2 * mH * dstate_dt[c_offset+2] - **mMechTorque + **mElecTorque;

	// residual function for mechanical theta
	resid[c_offset+3] = dstate_dt[c_offset+3] - mBase_OmMech * state[c_offset+2];

	// residual function of currents
	resid[c_offset+4] = state[c_offset+4] * mLd_t - state[c_offset+1] + (**mVdq0)(1,0);
	resid[c_offset+5] = state[c_offset+5] * mLq_t - (**mVdq0)(0,0) + state[c_offset];

	// add currents to residual funcion of nodes
	**mIntfCurrent = mBase_I * mDq0ToAbc * **mIdq0;
	resid[matrixNodeIndex(0, 0)] -= (**mIntfCurrent)(0, 0);
	resid[matrixNodeIndex(0, 1)] -= (**mIntfCurrent)(1, 0);
	resid[matrixNodeIndex(0, 2)] -= (**mIntfCurrent)(2, 0);
	off[1] += 6;
}

void EMT::Ph3::SynchronGenerator4OrderVBR::daeJacobian(double current_time, const double state[], 
	const double dstate_dt[], SUNMatrix jacobian, double cj, std::vector<int>& off) {

	// TODO

}

void EMT::Ph3::SynchronGenerator4OrderVBR::daePostStep(double Nexttime, const double state[], 
	const double dstate_dt[], int& offset) {
	
	(**mEdq0_t)(0,0) = state[offset];
	(**mEdq0_t)(1,0) = state[offset+1];
	**mOmMech = state[offset+2];
	**mThetaMech = state[offset+3];

	// get transformation matrix
	mAbcToDq0 = get_parkTransformMatrix();
	mDq0ToAbc = get_inverseParkTransformMatrix();

	(**mIntfVoltage)(0, 0) = state[matrixNodeIndex(0, 0)];
	(**mIntfVoltage)(1, 0) = state[matrixNodeIndex(0, 1)];
	(**mIntfVoltage)(2, 0) = state[matrixNodeIndex(0, 2)];
	**mVdq0 = mAbcToDq0 * **mIntfVoltage / mBase_V;

	// calculate dq0 and abc currents
	//(**mIdq0)(0,0) = ((**mEdq0_t)(1,0) - (**mVdq0)(1,0)) / mLd_t;
	//(**mIdq0)(1,0) = ((**mVdq0)(0,0) - (**mEdq0_t)(0,0)) / mLq_t;
	(**mIdq0)(0,0) = state[offset+4];
	(**mIdq0)(1,0) = state[offset+5];
	**mIntfCurrent = mBase_I * mDq0ToAbc * **mIdq0;

	offset+=6;
}
