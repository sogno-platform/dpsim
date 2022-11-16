/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_SynchronGenerator4OrderDCIM.h>

using namespace CPS;

SP::Ph1::SynchronGenerator4OrderDCIM::SynchronGenerator4OrderDCIM
    (const String & uid, const String & name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel),
	mEdq_t(mAttributes->create<Matrix>("Edq_t")) {

	//
	setTerminalNumber(1);

	// model variables
	**mEdq_t = Matrix::Zero(2,1);
}

SP::Ph1::SynchronGenerator4OrderDCIM::SynchronGenerator4OrderDCIM
	(const String & name, Logger::Level logLevel)
	: SynchronGenerator4OrderDCIM(name, name, logLevel) {
}

void SP::Ph1::SynchronGenerator4OrderDCIM::specificInitialization() {

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
	mCq = 2 * mTimeStep / (2 * mTd0_t + mTimeStep);

    // Initialize matrix of state representation
	mA = Matrix::Zero(2,2);
	mB = Matrix::Zero(2,2);
	mC = Matrix::Zero(2,1);
	calculateStateMatrix();

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model SPecific initialization finished ---",

		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0)
	);
	mSLog->flush();
}

void SP::Ph1::SynchronGenerator4OrderDCIM::calculateStateMatrix() {
	Real Td_t =  mTd0_t * (mLd_t / mLd);
	Real Tq_t =  mTq0_t * (mLq_t / mLq);
	mA << -1. / Tq_t,          0,
              0     ,      -1 / Td_t;
	mB <<	(1. / Tq_t) * (mLq-mLq_t) / mLq,	0.0,
			0.0, 								(1. / Td_t) * (mLd-mLd_t) / mLd;
	mC <<	0,
	   		(1. / Td_t) * **mEf * (mLd_t / mLd);
}

void SP::Ph1::SynchronGenerator4OrderDCIM::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
}

void SP::Ph1::SynchronGenerator4OrderDCIM::stepInPerUnit() {
	if (mSimTime>0.0) {
		// calculate mechanical variables at t=k+1 with forward euler
		**mElecTorque = ((**mVdq)(0,0) * (**mIdq)(0,0) + (**mVdq)(1,0) * (**mIdq)(1,0));
		**mOmMech = **mOmMech + mTimeStep * (1. / (2. * mH) * (**mMechTorque - **mElecTorque));
		**mThetaMech = **mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);
		**mDelta = **mDelta + mTimeStep * (**mOmMech - 1.) * mBase_OmMech;
	}

	// get transformation matrix
	mDqToComplexA = get_DqToComplexATransformMatrix();
	mComplexAToDq = mDqToComplexA.transpose();

	// calculate Edq at t=k+1. Assumption: Vdq(k) = Vdq(k+1)
	(**mEdq_t) = Math::StateSpaceTrapezoidal(**mEdq_t, mA, mB, mC, mTimeStep, **mVdq);

	// armature currents for at t=k+1
	(**mIdq)(0,0) = ((**mEdq_t)(1,0) - (**mVdq)(1,0) ) / mLd_t;
	(**mIdq)(1,0) = ((**mVdq)(0,0) - (**mEdq_t)(0,0) ) / mLq_t;

	// convert currents into the abc domain
	Matrix Ia = mDqToComplexA * **mIdq;
	(**mIntfCurrent)(0,0) = Complex(Ia(0,0), Ia(1,0)) * mBase_I_RMS;
}

void SP::Ph1::SynchronGenerator4OrderDCIM::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0), (**mIntfCurrent)(0, 0));
}

void SP::Ph1::SynchronGenerator4OrderDCIM::mnaCompPostStep(const Matrix& leftVector) {
	// update armature voltage
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	Matrix Vabc = Matrix::Zero(2,1);
	Vabc << (**mIntfVoltage)(0, 0).real(), (**mIntfVoltage)(0, 0).imag();
	**mVdq = mComplexAToDq * Vabc / mBase_V_RMS;

    mSimTime = mSimTime + mTimeStep;
}

Matrix SP::Ph1::SynchronGenerator4OrderDCIM::get_DqToComplexATransformMatrix() {
	Matrix dqToComplexA(2, 2);
	dqToComplexA <<
		cos(**mThetaMech - mBase_OmMech * mSimTime),	-sin(**mThetaMech - mBase_OmMech * mSimTime),
		sin(**mThetaMech - mBase_OmMech * mSimTime),	cos(**mThetaMech - mBase_OmMech * mSimTime);

	return dqToComplexA;
}
