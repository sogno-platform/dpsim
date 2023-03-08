/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator4OrderPCM.h>

using namespace CPS;

DP::Ph1::SynchronGenerator4OrderPCM::SynchronGenerator4OrderPCM
    (const String& uid, const String& name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel) {

	mPhaseType = PhaseType::Single;
	setTerminalNumber(1);

	/// initialize attributes
	mEdq = Attribute<Matrix>::create("Edq", mAttributes);
	mNumIter = Attribute<Int>::create("NIterations", mAttributes, 0);

	// Initialize matrix
	**mEdq = Matrix::Zero(2,1);
	mEdq_pred = Matrix::Zero(2,1);
	mEdq_corr = Matrix::Zero(2,1);
}

DP::Ph1::SynchronGenerator4OrderPCM::SynchronGenerator4OrderPCM
	(const String& name, Logger::Level logLevel)
	: SynchronGenerator4OrderPCM(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator4OrderPCM::clone(const String& name) {
	auto copy = SynchronGenerator4OrderPCM::make(name, mLogLevel);

	return copy;
}

void DP::Ph1::SynchronGenerator4OrderPCM::specificInitialization() {
	// calculate state representation matrix
	calculateStateMatrix();

	// initial voltage behind the transient reactance in the dq0 reference frame
	(**mEdq)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	// initialize transformation matrix dp->dq
	mDpToDq = Matrix::Zero(1,2);

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nMax number of iterations: {:d}"
		"\nTolerance: {:f}"
		"\n--- Model specific initialization finished ---",

		(**mEdq)(0,0),
		(**mEdq)(1,0),
		mMaxIter,
		mTolerance
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator4OrderPCM::calculateStateMatrix() {
	// Initialize matrices of state representation of predictor step
	if (mNumericalMethod == CPS::NumericalMethod::Euler) {
		mA_euler = Matrix::Zero(2,2);
		mA_euler << 1 - mTimeStep / mTq0_t,		0.0,
	      							   0.0,		1 - mTimeStep / mTd0_t;
		mB_euler = Matrix::Zero(2,2);
		mB_euler << 								0.0,	(mLq - mLq_t) * mTimeStep / mTq0_t,
		  			- (mLd - mLd_t) * mTimeStep / mTd0_t, 		0.0;
		mC_euler = Matrix::Zero(2,1);
		mC_euler <<   				0.0,
	  				(mTimeStep / mTd0_t);
	} else if (mNumericalMethod == CPS::NumericalMethod::Trapezoidal) {
		// TODO: State space model applied here is different than the Euler representation.
		// This needs harmonisation or further documentation.
		mC_trap = Matrix::Zero(4,1);
		mStates_trap_prev = Matrix::Zero(6,1);
		mStates_trap = Matrix::Zero(4,1);

		Real Ad = mTimeStep * (mLq - mLq_t) / (2 * mTq0_t + mTimeStep);
		Real Bd = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);

		Real Aq = - mTimeStep * (mLd - mLd_t) / (2 * mTd0_t + mTimeStep);
		Real Bq = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
		Real Cq = 2 * mTimeStep / (2 * mTd0_t + mTimeStep);

		mA_trap = Matrix::Zero(4,4);
		mA_trap <<	1,	0,		0,		-Ad,
					0,	1,		-Aq,	0,
					0,	-1,		mLd_t,	0,
					1,	0,		0,		mLq_t;
		mA_trap_inv = mA_trap.inverse();

		mB_trap = Matrix::Zero(4,6);
		mB_trap <<	Bd,		0,		0,		Ad,		0,		0,
					0,		Bq,		Aq,		0,		0,		0,
					0,		0,		0,		0,		0,		-1,
					0,		0,		0,		0,		1,		0;

		mC_trap = Matrix::Zero(4,1);
		mC_trap <<	0,	Cq,	0,	0;
	}

	// Initialize matrix of state representation of corrector step
	mA_prev = Matrix::Zero(2,2);
	mA_prev <<  1 - mTimeStep / (2 * mTq0_t),	0.0,
	       								 0.0,	1 - mTimeStep / (2 * mTd0_t);
	mA_corr = Matrix::Zero(2,2);
	mA_corr << - mTimeStep / (2 * mTq0_t),	0.0,
	    	   0.0, - mTimeStep / (2 * mTd0_t);
	mB_corr = Matrix::Zero(2,2);
	mB_corr <<	0.0, (mLq - mLq_t) * mTimeStep / (2 * mTq0_t),
				- (mLd - mLd_t) * mTimeStep / (2 * mTd0_t), 0.0;

	mC_corr = Matrix::Zero(2,1);
	mC_corr <<   				0.0,
	  			(mTimeStep / mTd0_t);
}

void DP::Ph1::SynchronGenerator4OrderPCM::stepInPerUnit() {
	// set number of iterations equal to zero
	**mNumIter = 0;

	// prediction of mechanical vars
	if (mSimTime > 0.0) {
		// calculate electrical torque at t=k
		**mElecTorque = (**mVdq)(0,0) * (**mIdq)(0,0) + (**mVdq)(1,0) * (**mIdq)(1,0);

		// predict mechanical variables at t=k+1 (forward euler)
		mOmMech_pred = **mOmMech + mTimeStep / (2 * mH) * (**mMechTorque - **mElecTorque);
		mDelta_pred = **mDelta + mTimeStep * mBase_OmMech * (**mOmMech - 1);
		mThetaMech_pred = **mThetaMech + mTimeStep * **mOmMech * mBase_OmMech;
	} else {
		// prediction by assuming constant mechanical vars at t=0
		// TODO: add further explanatory comment why this is required
		mOmMech_pred = **mOmMech;
		mDelta_pred = **mDelta;
		mThetaMech_pred = **mThetaMech;
	}

	// prediction of electrical vars
	if (mNumericalMethod == CPS::NumericalMethod::Euler) {
		// predict emf at t=k+1 (forward euler)
		mEdq_pred = mA_euler * (**mEdq) + mB_euler * **mIdq + mC_euler * mEf;

		// calculate stator currents at t=k+1 (assuming Vdq(k+1)=Vdq(k))
		mIdq_pred(0,0) = (mEdq_pred(1,0) - (**mVdq)(1,0) ) / mLd_t;
		mIdq_pred(1,0) = ((**mVdq)(0,0) - mEdq_pred(0,0) ) / mLq_t;
	} else if (mNumericalMethod == CPS::NumericalMethod::Trapezoidal) {
		// predict emfs and stator currents at t=k+1 (trapezoidal rule)
		mStates_trap_prev << **mEdq, **mIdq, **mVdq;
		mStates_trap = mA_trap_inv * mB_trap * mStates_trap_prev + mA_trap_inv * mC_trap * mEf;
		mEdq_pred << mStates_trap(0,0), mStates_trap(1,0);
		mIdq_pred << mStates_trap(2,0), mStates_trap(3,0);
	}

	// convert currents to dp domain
	mDpToDq(0,0) = Complex(cos(mThetaMech_pred - mBase_OmMech * mSimTime), sin(mThetaMech_pred - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(mThetaMech_pred - mBase_OmMech * mSimTime - PI/2.), sin(mThetaMech_pred - mBase_OmMech * mSimTime - PI/2.));
	(**mIntfCurrent)(0,0) = (mDpToDq * mIdq_pred)(0,0) * mBase_I_RMS;
}

void DP::Ph1::SynchronGenerator4OrderPCM::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0,0), (**mIntfCurrent)(0, 0));
}

void DP::Ph1::SynchronGenerator4OrderPCM::correctorStep() {
	// TODO: The naming of variables becomes confusing.
	// Probably better to use `PrevIter` and `CurrIter`
	// variables instead of misusing `_pred` variables or `mVdq`.

	// increase number of iterations
	**mNumIter = **mNumIter + 1;

	// correction of mechanical vars
	if (mSimTime>0.0) {
		// calculate electrical torque at t=k+1
		mElecTorque_corr = (**mVdq)(0,0) * (mIdq_pred)(0,0) + (**mVdq)(1,0) * (mIdq_pred)(1,0);

		// correct mechanical variables at t=k+1 (trapezoidal rule)
		mOmMech_corr = **mOmMech + mTimeStep / (4. * mH) * (2 * **mMechTorque - **mElecTorque - mElecTorque_corr);
		mDelta_corr = **mDelta + mTimeStep / 2. * mBase_OmMech * (**mOmMech + mOmMech_pred - 2);
		// CHECK: For mThetaMech_corr use mOmMech_corr already?
		mThetaMech_corr = **mThetaMech + mTimeStep / 2. *(**mOmMech + mOmMech_pred) * mBase_OmMech;
	} else {
		// correction by assuming constant mechanical vars at t=0
		// TODO: add further explanatory comment why this is required
		mElecTorque_corr = **mElecTorque;
		mOmMech_corr = mOmMech_pred;
		mDelta_corr = mDelta_pred;
		mThetaMech_corr = mThetaMech_pred;
	}

	// correction of electrical vars
	// correct emf at t=k+1 (trapezoidal rule)
	mEdq_corr = mA_prev * **mEdq + mA_corr * mEdq_pred + mB_corr * (**mIdq + mIdq_pred) + mC_corr * mEf;

	// calculate corrected stator currents at t=k+1 (assuming Vdq(k+1)=VdqPrevIter(k+1))
	mIdq_corr(0,0) = (mEdq_corr(1,0) - (**mVdq)(1,0) ) / mLd_t;
	mIdq_corr(1,0) = ((**mVdq)(0,0) - mEdq_corr(0,0) ) / mLq_t;

	// convert corrected currents to dp domain
	mDpToDq(0,0) = Complex(cos(mThetaMech_corr - mBase_OmMech * mSimTime), sin(mThetaMech_corr - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(mThetaMech_corr - mBase_OmMech * mSimTime - PI/2.), sin(mThetaMech_corr - mBase_OmMech * mSimTime - PI/2.));
	(**mIntfCurrent)(0,0) = (mDpToDq * mIdq_corr)(0,0) * mBase_I_RMS;

	// stamp currents
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::SynchronGenerator4OrderPCM::updateVoltage(const Matrix& leftVector) {
	// TODO: The naming of variables becomes confusing.
	// Probably better to use `PrevIter` and `CurrIter`
	// variables instead of misusing `_pred` variables or `mVdq`.

	mVdq_prev = **mVdq;
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

	// convert armature voltage into dq reference frame
	MatrixComp Vabc_ = (**mIntfVoltage)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	if (**mNumIter == 0)
		**mVdq = parkTransform(mThetaMech_pred, Vabc) / mBase_V_RMS;
	else
		**mVdq = parkTransform(mThetaMech_corr, Vabc) / mBase_V_RMS;

	mOmMech_pred = mOmMech_corr;
	mDelta_pred = mDelta_corr;
	mThetaMech_pred = mThetaMech_corr;
	mIdq_pred = mIdq_corr;
	mEdq_pred = mEdq_corr;
}

bool DP::Ph1::SynchronGenerator4OrderPCM::requiresIteration() {
	if (**mNumIter == 0)
		// if no corrector step has been performed yet
		return true;

	Matrix voltageDifference = **mVdq - mVdq_prev;
	if (Math::abs(voltageDifference(0,0)) > mTolerance || Math::abs(voltageDifference(1,0)) > mTolerance) {
		if (**mNumIter >= mMaxIter) {
			return false;
		} else {
			return true;
		}
	} else {
		return false;
	}
}

void DP::Ph1::SynchronGenerator4OrderPCM::mnaPostStep(const Matrix& leftVector) {
	// update variables
	**mOmMech = mOmMech_corr;
	**mThetaMech = mThetaMech_corr;
	**mDelta = mDelta_corr;
	**mEdq = mEdq_corr;
	**mIdq = mIdq_corr;
}
