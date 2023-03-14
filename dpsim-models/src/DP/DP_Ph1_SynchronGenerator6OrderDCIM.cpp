/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator6OrderDCIM.h>

using namespace CPS;

DP::Ph1::SynchronGenerator6OrderDCIM::SynchronGenerator6OrderDCIM
    (String uid, String name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel),
	mEdq_t(mAttributes->create<Matrix>("Edq_t")),
	mEdq_s(mAttributes->create<Matrix>("Edq_s")) {

	setTerminalNumber(1);

	// model variables
	**mEdq_t = Matrix::Zero(2,1);
	**mEdq_s = Matrix::Zero(2,1);

	//
	mShiftVector = Matrix::Zero(3,1);
	mShiftVector << Complex(1., 0), SHIFT_TO_PHASE_B, SHIFT_TO_PHASE_C;
}

DP::Ph1::SynchronGenerator6OrderDCIM::SynchronGenerator6OrderDCIM
	(String name, Logger::Level logLevel)
	: SynchronGenerator6OrderDCIM(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator6OrderDCIM::clone(String name) {

	auto copy = SynchronGenerator6OrderDCIM::make(name, mLogLevel);
	return copy;
}

void DP::Ph1::SynchronGenerator6OrderDCIM::specificInitialization() {

	// initial voltage behind the transient reactance in the dq reference frame
	(**mEdq_t)(0,0) = (mLq - mLq_t) * (**mIdq)(1,0);
	(**mEdq_t)(1,0) = (**mEf) - (mLd - mLd_t) * (**mIdq)(0,0);

	// initial dq behind the subtransient reactance in the dq reference frame
	(**mEdq_s)(0,0) = (**mVdq)(0,0) - mLq_s * (**mIdq)(1,0);
	(**mEdq_s)(1,0) = (**mVdq)(1,0) + mLd_s * (**mIdq)(0,0);

	//
	mStates = Matrix::Zero(6,1);
	mStates << **mEdq_s, **mEdq_t, **mIdq;
	mStates_prev = Matrix::Zero(8,1);

	//
	mAd_t = mTimeStep * (mLq - mLq_t) / (2 * mTq0_t + mTimeStep);
	mBd_t = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);
	mAq_t = - mTimeStep * (mLd - mLd_t) / (2 * mTd0_t + mTimeStep);
	mBq_t = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
	mDq_t = 2 * mTimeStep / (2 * mTd0_t + mTimeStep);

	//
	mAd_s = mTimeStep * (mLq_t - mLq_s) / (2 * mTq0_s + mTimeStep);
	mBd_s = mTimeStep / (2 * mTq0_s + mTimeStep);
	mCd_s = (2 * mTq0_s - mTimeStep) / (2 * mTq0_s + mTimeStep);
	mAq_s = mTimeStep * (mLd_t - mLd_s) / (2 * mTd0_s + mTimeStep);
	mBq_s = mTimeStep / (2 * mTd0_s + mTimeStep);
	mCq_s = (2 * mTd0_s - mTimeStep) / (2 * mTd0_s + mTimeStep);

    // Initialize matrix of state representation
	mA = Matrix::Zero(6,6);
	mA <<	1,		0,		-mBd_s,		0,			0,		-mAd_s,
			0,		1,		0,		-mBq_s,		mAq_s,		0,
			0,		0,		1,			0,			0,		-mAd_t,
			0,		0,		0,			1,		-mAq_t,		0,
			0,		-1,		0,			0,		mLd_s,		0,
			1,		0,		0,			0,			0,		mLq_s;
	mA_inv = mA.inverse();

	mB = Matrix::Zero(6,8);
	mB <<	mCd_s,	0,		mBd_s,		0,		0,		mAd_s,	0,	0,
			0,		mCq_s,	0,			mBq_s,	-mAq_s,		0,	0,	0,
			0,		0,		mBd_t,		0,		0,		mAd_t,	0,	0,
			0,		0,		0,			mBq_t,	mAq_t,		0,	0,	0,
			0,		0,		0,			0,		0,			0,	0,	-1,
			0,		0,		0,			0,		0,			0,	1,	0;

	mC = Matrix::Zero(6,1);
	mC <<	0,	0,	0,	mDq_t,	0,	0;

	mSLog->info(
		"\n--- Model specific initialization  ---"
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

void DP::Ph1::SynchronGenerator6OrderDCIM::mnaCompApplySystemMatrixStamp(Matrix& systemMatrix) {
}

void DP::Ph1::SynchronGenerator6OrderDCIM::stepInPerUnit() {
	if (mSimTime>0.0) {
		// calculate mechanical variables at t=k+1 with forward euler
		**mElecTorque = ((**mVdq)(0,0) * (**mIdq)(0,0) + (**mVdq)(1,0) * (**mIdq)(1,0));
		**mOmMech = **mOmMech + mTimeStep * (1. / (2. * mH) * (**mMechTorque - **mElecTorque));
		**mThetaMech = **mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);
		**mDelta = **mDelta + mTimeStep * (**mOmMech - 1.) * mBase_OmMech;
	}

	// calculate Edq and Idq at t=k+1. Assumption: Vdq(k) = Vdq(k+1)
	mStates_prev << mStates, **mVdq;
	mStates = mA_inv * mB * mStates_prev + mA_inv * mC * (**mEf);
	**mEdq_s <<	mStates(0,0), mStates(1,0);
	**mEdq_t <<	mStates(2,0), mStates(3,0);
	**mIdq << mStates(4,0), mStates(5,0);

	// convert currents into the abc reference frame
	// TODO
	//mDpToDq(0,0) = Complex(cos(**mThetaMech - mBase_OmMech * mSimTime), sin(**mThetaMech - mBase_OmMech * mSimTime));
	//mDpToDq(0,1) = -Complex(cos(**mThetaMech - mBase_OmMech * mSimTime - PI/2.), sin(**mThetaMech - mBase_OmMech * mSimTime - PI/2.));
	//(**mIntfCurrent)(0,0) = (mDpToDq * **mIdq)(0,0) * mBase_I_RMS;
}

void DP::Ph1::SynchronGenerator6OrderDCIM::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0), (**mIntfCurrent)(0, 0));
}

void DP::Ph1::SynchronGenerator6OrderDCIM::mnaCompPostStep(const Matrix& leftVector) {

	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

	// convert armature voltage into dq reference frame
	MatrixComp Vabc_ = (**mIntfVoltage)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	**mVdq = parkTransform(**mThetaMech, Vabc) / mBase_V_RMS;

    mSimTime = mSimTime + mTimeStep;
}

Matrix DP::Ph1::SynchronGenerator6OrderDCIM::parkTransform(Real theta, const Matrix& abcVector) {
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
