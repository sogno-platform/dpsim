/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_SynchronGenerator6OrderIter.h>

using namespace CPS;

DP::Ph1::SynchronGenerator6OrderIter::SynchronGenerator6OrderIter
    (String uid, String name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel) {

	mPhaseType = PhaseType::Single;
	setTerminalNumber(1);

	// model flags (deprecated)
	mVoltageForm = false;
	
	// model variables
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	// Initialize matrix
	mIdq_pred = Matrix::Zero(2,1);
	mIdq_corr = Matrix::Zero(2,1);
	mEdq = Matrix::Zero(4,1);
	mEdq_pred = Matrix::Zero(4,1);
	mEdq_corr = Matrix::Zero(4,1);

	//
	mShiftVector = Matrix::Zero(3,1);
	mShiftVector << Complex(1., 0), SHIFT_TO_PHASE_B, SHIFT_TO_PHASE_C;

    // Register attributes
	addAttribute<Int>("NIterations", &mNumIter , Flags::read);
	addAttribute<Matrix>("Edq0_t", &mEdq, Flags::read);
}

DP::Ph1::SynchronGenerator6OrderIter::SynchronGenerator6OrderIter
	(String name, Logger::Level logLevel)
	: SynchronGenerator6OrderIter(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator6OrderIter::clone(String name) {
	auto copy = SynchronGenerator6OrderIter::make(name, mLogLevel);
	
	return copy;
}

void DP::Ph1::SynchronGenerator6OrderIter::specificInitialization() {

	// Initialize matrix of state representation
	mA = Matrix::Zero(4,4);
	mA_prev = Matrix::Zero(4,4);
	mA_corr = Matrix::Zero(4,4);
	mB = Matrix::Zero(4,2);
	mB_corr = Matrix::Zero(4,2);
	mC = Matrix::Zero(4,1);
	calculateStateMatrix();

	// initial voltage behind the transient reactance in the dq0 reference frame
	mEdq(0,0) = (mLq - mLq_t) * mIdq(1,0);
	mEdq(1,0) = - (mLd- mLd_t) * mIdq(0,0) + mEf;
	mEdq(2,0) = mEdq(0,0) + (mLq_t - mLq_s) * mIdq(1,0);
	mEdq(3,0) = mEdq(1,0) - (mLd_t - mLd_s) * mIdq(0,0);

	// initialize transformation matrix dp->dq
	mDpToDq = Matrix::Zero(1,2);

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nInitial Ed_s (per unit): {:f}"
		"\nInitial Eq_s (per unit): {:f}"
		"\nMax number of iterations: {:d}"
		"\nTolerance: {:f}"
		"\n--- Model specific initialization finished ---",

		mEdq(0,0),
		mEdq(1,0),
		mEdq(2,0),
		mEdq(3,0),
		mMaxIter,
		mTolerance
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator6OrderIter::calculateStateMatrix() {	
	mA << 1 - mTimeStep / mTq0_t,	0.0,	0.0,	0.0,
	      0.0,		1 - mTimeStep / mTd0_t,	0.0,	0.0,
		  mTimeStep / mTq0_s,	0.0,	1 - mTimeStep / mTq0_s, 0.0,
		  0.0,	mTimeStep / mTd0_s,		0.0,	1 - mTimeStep / mTd0_s;
	mA_prev <<  1 - mTimeStep / (2 * mTq0_t),	0.0,	0.0,	0.0,
	       		0.0,		1 - mTimeStep / (2 * mTd0_t),	0.0,	0.0,
		  		mTimeStep / (2 * mTq0_s),	0.0,	1 - mTimeStep / (2 * mTq0_s), 0.0,
		  		0.0,	mTimeStep / (2 * mTd0_s),		0.0,	1 - mTimeStep / (2 * mTd0_s);
	mA_corr << 	- mTimeStep / (2 * mTq0_t),	0.0,	0.0,	0.0,
	       		0.0, - mTimeStep / (2 * mTd0_t),	0.0,	0.0,
		  		mTimeStep / (2 * mTq0_s),	0.0,	- mTimeStep / (2 * mTq0_s), 0.0,
		  		0.0,	mTimeStep / (2 * mTd0_s),		0.0,	- mTimeStep / (2 * mTd0_s);
	mB << 0.0, (mLq - mLq_t) * mTimeStep / mTq0_t,
		  - (mLd - mLd_t) * mTimeStep / mTd0_t, 0.0,
		  0.0, (mLq_t - mLq_s) * mTimeStep / mTq0_s,
		  - (mLd_t - mLd_s) * mTimeStep / mTd0_s, 0.0;
	mB_corr <<  0.0, (mLq - mLq_t) * mTimeStep / (2 * mTq0_t),
			- (mLd - mLd_t) * mTimeStep / (2 * mTd0_t), 0.0,
	  		0.0, (mLq_t - mLq_s) * mTimeStep / (2 * mTq0_s),
	  		- (mLd_t - mLd_s) * mTimeStep / (2 * mTd0_s), 0.0;
	mC <<   0.0,
	  		(mTimeStep / mTd0_t),
			0.0,
			0.0;
}

void DP::Ph1::SynchronGenerator6OrderIter::stepInPerUnit() {
	// set number of iteratios equal to zero
	mNumIter = 0;

	// Predictor step (backward euler)
	if (mSimTime>0.0) {
		// calculate electrical torque at t=k-1
		mElecTorque = mVdq(0,0) * mIdq(0,0) + mVdq(1,0) * mIdq(1,0);
		// predict mechanical variables at t=k
		mOmMech_pred = mOmMech + mTimeStep / (2 * mH) * (mMechTorque - mElecTorque);
		mDelta_pred = mDelta + mTimeStep * mBase_OmMech * (mOmMech - 1);
		mThetaMech_pred = mThetaMech + mTimeStep * mOmMech * mBase_OmMech;
	} else {
		mOmMech_pred = mOmMech;
		mDelta_pred = mDelta;
		mThetaMech_pred = mThetaMech;
	}

	//predict voltage behind transient reactance 
	mEdq_pred = mA * mEdq + mB * mIdq + mC * mEf;

	// predict armature currents for at t=k+1
	mIdq_pred(0,0) = (mEdq_pred(3,0) - mVdq(1,0) ) / mLd_s;
	mIdq_pred(1,0) = (mVdq(0,0) - mEdq_pred(2,0) ) / mLq_s;

	// convert currents into the abc reference frame
	mDpToDq(0,0) = Complex(cos(mThetaMech_pred - mBase_OmMech * mSimTime), sin(mThetaMech_pred - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(mThetaMech_pred - mBase_OmMech * mSimTime - PI/2.), sin(mThetaMech_pred - mBase_OmMech * mSimTime - PI/2.));
	mIntfCurrent(0,0) = (mDpToDq * mIdq_pred)(0,0) * mBase_I_RMS;
}

void DP::Ph1::SynchronGenerator6OrderIter::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mIntfCurrent(0, 0));
}

void DP::Ph1::SynchronGenerator6OrderIter::correctorStep() {
	// corrector step (trapezoidal rule)
	mNumIter = mNumIter + 1;

	if (mSimTime>0.0) {
		// calculate electrical torque at t=k
		mElecTorque_corr = mVdq(0,0) * mIdq(0,0) + mVdq(1,0) * mIdq(1,0);
		// correct mechanical variables at t=k
		mOmMech_corr = mOmMech + mTimeStep / (4. * mH) * (2 * mMechTorque - mElecTorque - mElecTorque_corr);
		mDelta_corr = mDelta + mTimeStep / 2. * mBase_OmMech * (mOmMech + mOmMech_pred - 2);
		mThetaMech_corr = mThetaMech + mTimeStep / 2. *(mOmMech + mOmMech_pred) * mBase_OmMech;
	} else {
		mElecTorque_corr = mElecTorque;
		mOmMech_corr = mOmMech_pred;
		mDelta_corr = mDelta_pred;
		mThetaMech_corr = mThetaMech_pred;
	}

	//predict voltage behind transient reactance
	mEdq_corr = mA_prev * mEdq + mA_corr * mEdq_pred + mB_corr * (mIdq + mIdq_pred) + mC * mEf   ;

	// armature currents for at t=k+1
	mIdq_corr(0,0) = (mEdq_corr(1,0) - mVdq(1,0) ) / mLd_t;
	mIdq_corr(1,0) = (mVdq(0,0) - mEdq_corr(0,0) ) / mLq_t;

	// convert currents into the abc reference frame
	mDpToDq(0,0) = Complex(cos(mThetaMech_corr - mBase_OmMech * mSimTime), sin(mThetaMech_corr - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(mThetaMech_corr - mBase_OmMech * mSimTime - PI/2.), sin(mThetaMech_corr - mBase_OmMech * mSimTime - PI/2.));
	mIntfCurrent(0,0) = (mDpToDq * mIdq_corr)(0,0) * mBase_I_RMS;

	// stamp currents
	mnaApplyRightSideVectorStamp(mRightVector);
}

void DP::Ph1::SynchronGenerator6OrderIter::updateVoltage(const Matrix& leftVector) {
	mVdq_prev = mVdq;
	mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	
	// convert armature voltage into dq reference frame
	MatrixComp Vabc_ = mIntfVoltage(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	if (mNumIter==0) {
		mVdq = parkTransform(mThetaMech_pred, Vabc) / mBase_V_RMS;
	} else {
		mVdq = parkTransform(mThetaMech_corr, Vabc) / mBase_V_RMS;
	}
}

bool DP::Ph1::SynchronGenerator6OrderIter::checkVoltageDifference() {
	if (mNumIter==0) {
		// if no corrector step has been performed yet
		return true;
	}

	Matrix voltageDifference = mVdq - mVdq_prev;
	if (Math::abs(voltageDifference(0,0)) > mTolerance || Math::abs(voltageDifference(1,0)) > mTolerance) {
		if (mNumIter == mMaxIter) {
			return false;
		} else {
			// set predicted values equal to corrected values for the next iteration
			mOmMech_pred = mOmMech_corr;
			mDelta_pred = mDelta_corr;
			mThetaMech_pred= mDelta_corr;
			mIdq_pred = mIdq_corr;
			mEdq_pred = mEdq_corr;
			
			return true;
		}
	} else {
		return false;
	}
}

void DP::Ph1::SynchronGenerator6OrderIter::mnaPostStep(const Matrix& leftVector) {
	// update variables
	mOmMech = mOmMech_corr;
	mThetaMech = mThetaMech_corr;
	mDelta = mDelta_corr;
	mEdq = mEdq_corr;
	mIdq = mIdq_corr;
}

Matrix DP::Ph1::SynchronGenerator6OrderIter::parkTransform(Real theta, const Matrix& abcVector) {
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