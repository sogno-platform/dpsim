/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator6OrderPCM.h>

using namespace CPS;

DP::Ph1::SynchronGenerator6OrderPCM::SynchronGenerator6OrderPCM
    (const String& uid, const String& name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel),
	mEdq_t(mAttributes->create<Matrix>("Edq_t")),
	mEdq_s(mAttributes->create<Matrix>("Edq_s")) {

	mPhaseType = PhaseType::Single;
	setTerminalNumber(1);

	/// initialize attributes
	mNumIter = mAttributes->create<Int>("NIterations", 0);

	// model variables
	**mEdq_t = Matrix::Zero(2,1);
	**mEdq_s = Matrix::Zero(2,1);
}

DP::Ph1::SynchronGenerator6OrderPCM::SynchronGenerator6OrderPCM
	(const String& name, Logger::Level logLevel)
	: SynchronGenerator6OrderPCM(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator6OrderPCM::clone(const String& name) {
	auto copy = SynchronGenerator6OrderPCM::make(name, mLogLevel);

	return copy;
}

void DP::Ph1::SynchronGenerator6OrderPCM::specificInitialization() {

	// calculate state representation matrix
	calculateStateMatrix();

	// initial voltage behind the transient reactance in the dq0 reference frame
	(**mEdq_t)(0,0) = (mLq - mLq_t) * (**mIdq)(1,0);
	(**mEdq_t)(1,0) = - (mLd- mLd_t) * (**mIdq)(0,0) + (**mEf);
	(**mEdq_s)(0,0) = (**mEdq_t)(0,0) + (mLq_t - mLq_s) * (**mIdq)(1,0);
	(**mEdq_s)(1,0) = (**mEdq_t)(1,0) - (mLd_t - mLd_s) * (**mIdq)(0,0);

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

		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0),
		(**mEdq_s)(0,0),
		(**mEdq_s)(1,0),
		mMaxIter,
		mTolerance
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator6OrderPCM::calculateStateMatrix() {
	// Initialize matrix of state representation of predictor step
	mA_euler = Matrix::Zero(4,4);
	mA_euler << 1 - mTimeStep / mTq0_t, 0.0,	0.0,	0.0,
    	  		0.0, 1 - mTimeStep / mTd0_t,	0.0,	0.0,
	  	 		mTimeStep / mTq0_s, 0.0, 1 - mTimeStep / mTq0_s, 0.0,
	  	  		0.0, mTimeStep / mTd0_s, 0.0,	1 - mTimeStep / mTd0_s;
	mB_euler = Matrix::Zero(4,2);
	mB_euler << 0.0, (mLq - mLq_t) * mTimeStep / mTq0_t,
	  			- (mLd - mLd_t) * mTimeStep / mTd0_t, 0.0,
	  			0.0, (mLq_t - mLq_s) * mTimeStep / mTq0_s,
	  			- (mLd_t - mLd_s) * mTimeStep / mTd0_s, 0.0;
	mC_euler = Matrix::Zero(4,1);
	mC_euler << 0.0,
  				(mTimeStep / mTd0_t),
				0.0,
				0.0;

	// Initialize matrix of state representation of corrector step
	mA_prev = Matrix::Zero(4, 4);
	mA_prev <<  1 - mTimeStep / (2 * mTq0_t),		0.0,				 0.0,	  	 0.0,
	       		0.0,		1 - mTimeStep / (2 * mTd0_t),				 0.0,	  	 0.0,
		  		mTimeStep / (2 * mTq0_s),	0.0,	1 - mTimeStep / (2 * mTq0_s), 	 0.0,
		  		0.0,	mTimeStep / (2 * mTd0_s),	0.0,	1 - mTimeStep / (2 * mTd0_s);
	mA_corr = Matrix::Zero(4,4);
	mA_corr << 	- mTimeStep / (2 * mTq0_t),	0.0,							0.0,	0.0,
	       		0.0, 						- mTimeStep / (2 * mTd0_t),		0.0,	0.0,
		  		mTimeStep / (2 * mTq0_s),			0.0,	- mTimeStep / (2 * mTq0_s), 0.0,
		  		0.0,	mTimeStep / (2 * mTd0_s),	0.0,		- mTimeStep / (2 * mTd0_s);

	mB_corr = Matrix::Zero(4,2);
	mB_corr <<  0.0, (mLq - mLq_t) * mTimeStep / (2 * mTq0_t),
			- (mLd - mLd_t) * mTimeStep / (2 * mTd0_t), 0.0,
	  		0.0, (mLq_t - mLq_s) * mTimeStep / (2 * mTq0_s),
	  		- (mLd_t - mLd_s) * mTimeStep / (2 * mTd0_s), 0.0;
	mC_corr = Matrix::Zero(4,1);
	mC_corr <<   0.0,
	  		(mTimeStep / mTd0_t),
			0.0,
			0.0;
}

void DP::Ph1::SynchronGenerator6OrderPCM::stepInPerUnit() {
	// set number of iteratios equal to zero
	**mNumIter = 0;

	// store values currently at t=k-1 for later use
	mEdqsPrevStep = **mEdq_s;
	mEdqtPrevStep = **mEdq_t;
	mIdqPrevStep = **mIdq;

	// prediction emf at t=k
	// TODO
	//auto mEdq = mA_euler * (**mEdq) + mB_euler * **mIdq + mC_euler * (**mEf);

	// predict armature currents for at t=k+1
	(**mIdq)(0,0) = ((**mEdq_s)(1,0) - (**mVdq)(1,0) ) / mLd_s;
	(**mIdq)(1,0) = ((**mVdq)(0,0) - (**mEdq_s)(0,0) ) / mLq_s;

	// convert currents into the abc reference frame
	mDpToDq(0,0) = Complex(cos(**mThetaMech - mBase_OmMech * mSimTime), sin(**mThetaMech - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(**mThetaMech - mBase_OmMech * mSimTime - PI/2.), sin(**mThetaMech - mBase_OmMech * mSimTime - PI/2.));
	(**mIntfCurrent)(0,0) = (mDpToDq * **mIdq)(0,0) * mBase_I_RMS;
}

void DP::Ph1::SynchronGenerator6OrderPCM::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0,0), (**mIntfCurrent)(0, 0));
}

void DP::Ph1::SynchronGenerator6OrderPCM::correctorStep() {
	// corrector step (trapezoidal rule)
	**mNumIter = **mNumIter + 1;

	// correction of electrical vars
	// correct emf at t=k+1 (trapezoidal rule)
	// TODO
	//(**mEdq) = mA_prev * mEdqtPrevStep + mA_corr * (**mEdq) + mB_corr * (mIdqPrevStep + **mIdq) + mC_corr * (**mEf);

	// calculate corrected stator currents at t=k+1 (assuming Vdq(k+1)=VdqPrevIter(k+1))
	(**mIdq)(0,0) = ((**mEdq_s)(1,0) - (**mVdq)(1,0) ) / mLd_t;
	(**mIdq)(1,0) = ((**mVdq)(0,0) - (**mEdq_s)(0,0) ) / mLq_t;

	// convert corrected currents to dp domain
	(**mIntfCurrent)(0,0) = (mDpToDq * **mIdq)(0,0) * mBase_I_RMS;

	// stamp currents
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::SynchronGenerator6OrderPCM::updateVoltage(const Matrix& leftVector) {
	// store voltage value currently at j-1 for later use
	mVdqPrevIter = **mVdq;

	//
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

	// convert armature voltage into dq reference frame
	MatrixComp Vabc_ = (**mIntfVoltage)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	**mVdq = parkTransform(**mThetaMech, Vabc) / mBase_V_RMS;
}

bool DP::Ph1::SynchronGenerator6OrderPCM::requiresIteration() {
	if (**mNumIter == 0)
		// if no corrector step has been performed yet
		return true;

	Matrix voltageDifference = **mVdq - mVdqPrevIter;
	if (Math::abs(voltageDifference(0,0)) > mTolerance || Math::abs(voltageDifference(1,0)) > mTolerance) {
		if (**mNumIter == mMaxIter) {
			return false;
		} else {
			return true;
		}
	} else {
		return false;
	}
}

void DP::Ph1::SynchronGenerator6OrderPCM::mnaCompPostStep(const Matrix& leftVector) {
}

