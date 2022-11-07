/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator4OrderDCIM.h>

using namespace CPS;

DP::Ph1::SynchronGenerator4OrderDCIM::SynchronGenerator4OrderDCIM
    (String uid, String name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel),
	mEdq_t(Attribute<Matrix>::create("Edq0_t", mAttributes)) {

	setTerminalNumber(1);

	// model variables
	**mEdq_t = Matrix::Zero(2,1);

	//
	mShiftVector = Matrix::Zero(3,1);
	mShiftVector << Complex(1., 0), SHIFT_TO_PHASE_B, SHIFT_TO_PHASE_C;

	// initialize transformation matrix dp->dq
	mDpToDq = Matrix::Zero(1,2);
}

DP::Ph1::SynchronGenerator4OrderDCIM::SynchronGenerator4OrderDCIM
	(String name, Logger::Level logLevel)
	: SynchronGenerator4OrderDCIM(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator4OrderDCIM::clone(String name) {
	
	auto copy = SynchronGenerator4OrderDCIM::make(name, mLogLevel);
	return copy;
}

void DP::Ph1::SynchronGenerator4OrderDCIM::specificInitialization() {

	// initial voltage behind the transient reactance in the dq reference frame
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	//
	mStates = Matrix::Zero(4,1);
	mStates << **mEdq_t, **mIdq;
	mStates_prev = Matrix::Zero(6,1);

	//
	mAd = mTimeStep * (mLq - mLq_t) / (2 * mTq0_t + mTimeStep);
	mBd = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);

	mAq = - mTimeStep * (mLd - mLd_t) / (2 * mTd0_t + mTimeStep);
	mBq = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
	mCq = 2 * mTimeStep * mEf / (2 * mTd0_t + mTimeStep);

    // Initialize matrix of state representation
	mA = Matrix::Zero(4,4);
	mA <<	1,	0,		0,		-mAd,
			0,	1,		-mAq,	0,
			0,	-1,		mLd_t,	0,
			1,	0,		0,		mLq_t;
	mA_inv = mA.inverse();
	
	mB = Matrix::Zero(4,6);
	mB <<	mBd,	0,		0,		mAd,	0,		0,
			0,		mBq,	mAq,	0,		0,		0,
			0,		0,		0,		0,		0,		-1,
			0,		0,		0,		0,		1,		0;

	mC = Matrix::Zero(4,1);
	mC <<	0,	mCq,	0,	0;
    
	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model DPecific initialization finished ---",

		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0)
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator4OrderDCIM::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
}

void DP::Ph1::SynchronGenerator4OrderDCIM::stepInPerUnit() {
	if (mSimTime>0.0) {
		// calculate mechanical variables at t=k+1 with forward euler
		**mOmMech = **mOmMech + mTimeStep * (1. / (2. * mH) * (**mMechTorque - **mElecTorque));
		**mThetaMech = **mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);
		**mDelta = **mDelta + mTimeStep * (**mOmMech - 1.) * mBase_OmMech;
        **mElecTorque = ((**mVdq)(0,0) * (**mIdq)(0,0) + (**mVdq)(1,0) * (**mIdq)(1,0));
	}

	// calculate Edq and Idq at t=k+1. Assumption: Vdq(k) = Vdq(k+1)
	mStates_prev << mStates, mVdq;
	mStates = mA_inv * mB * mStates_prev + mA_inv * mC * mEf;
	**mEdq_t <<	mStates(0,0), mStates(1,0);
	**mIdq << mStates(2,0), mStates(3,0);

	// convert currents into the abc reference frame
	mDpToDq(0,0) = Complex(cos(**mThetaMech - mBase_OmMech * mSimTime), sin(**mThetaMech - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(**mThetaMech - mBase_OmMech * mSimTime - PI/2.), sin(**mThetaMech - mBase_OmMech * mSimTime - PI/2.));
	(**mIntfCurrent)(0,0) = (mDpToDq * **mIdq)(0,0) * mBase_I_RMS;
}

void DP::Ph1::SynchronGenerator4OrderDCIM::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0), (**mIntfCurrent)(0, 0));
}

void DP::Ph1::SynchronGenerator4OrderDCIM::mnaPostStep(const Matrix& leftVector) {

	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	
	// convert armature voltage into dq reference frame
	MatrixComp Vabc_ = (**mIntfVoltage)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	**mVdq = parkTransform(**mThetaMech, Vabc) / mBase_V_RMS;

    mSimTime = mSimTime + mTimeStep;
}

Matrix DP::Ph1::SynchronGenerator4OrderDCIM::parkTransform(Real theta, const Matrix& abcVector) {
	Matrix dq0Vector(3, 1);
	Matrix dqVector(2, 1);
	Matrix abcToDq0(3, 3);

	// Park transform according to Kundur
	abcToDq0 <<
		 2./3.*cos(theta),	2./3.*cos(theta - 2.*PI/3.),  2./3.*cos(theta + 2.*PI/3.),
		-2./3.*sin(theta), -2./3.*sin(theta - 2.*PI/3.), -2./3.*sin(theta + 2.*PI/3.),
		 1./3., 			1./3., 						  1./3.;

	dq0Vector = abcToDq0 * abcVector;
	dqVector << dq0Vector(0,0), dq0Vector(1,0);
	return dqVector;
}