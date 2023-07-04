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

	mSGOrder = SGOrder::SG6bOrder;
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
	calculateStateSpaceMatrices();

	// initial voltage behind the transient reactance in the dq0 reference frame
	(**mEdq_t)(0,0) = (mLq - mLq_t) * (**mIdq)(1,0);
	(**mEdq_t)(1,0) = - (mLd- mLd_t) * (**mIdq)(0,0) + (**mEf);
	(**mEdq_s)(0,0) = (**mEdq_t)(0,0) + (mLq_t - mLq_s) * (**mIdq)(1,0);
	(**mEdq_s)(1,0) = (**mEdq_t)(1,0) - (mLd_t - mLd_s) * (**mIdq)(0,0);

	//
	mEdqts << **mEdq_t, **mEdq_s;

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nInitial Ed_s (per unit): {:f}"
		"\nInitial Eq_s (per unit): {:f}"
		"\nMax number of iterations: {:d}"
		"\nTolerance: {:f}"
		"\nSG Model: 6 Order (Anderson-Fouad) PCM"
		"\n--- Model specific initialization finished ---",

		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0),
		(**mEdq_s)(0,0),
		(**mEdq_s)(1,0),
		mMaxIter,
		mTolerance
	);
	mSLog->flush();

	mDomainInterface.setDPShiftFrequency(mBase_OmMech);
}

void DP::Ph1::SynchronGenerator6OrderPCM::calculateStateSpaceMatrices() {
	// auxiliar constants
	Real Bd_t = 1. / mTq0_t * (mLq - mLq_t);
	Real Bq_t = 1. / mTd0_t * (mLd - mLd_t);
	Real Bd_s = 1. / mTq0_s * (mLq_t - mLq_s);
	Real Bq_s = 1. / mTd0_s * (mLd_t - mLd_s);

	// Initialize matrices of state representation of predictor step
	mAStateSpace <<	-1. / mTq0_t, 0., 0., 0.,
              		0., -1. / mTd0_t, 0., 0.,
					1. / mTq0_s, 0., -1. / mTq0_s, 0.,
					0., 1. / mTd0_s, 0., -1. / mTd0_s;
	mBStateSpace <<	0., Bd_t,
					-Bq_t, 0.,
					0., Bd_s,
					-Bq_s, 0.;
	mCStateSpace <<	0.,
					1 / mTd0_t,
					0.,
					0.;

	// Precalculate trapezoidal based matrices (avoids redundant matrix inversions in correction steps)
	Math::calculateStateSpaceTrapezoidalMatrices(mAStateSpace, mBStateSpace, mCStateSpace, mTimeStep, mAdTrapezoidal, mBdTrapezoidal, mCdTrapezoidal);
}

void DP::Ph1::SynchronGenerator6OrderPCM::stepInPerUnit() {
	// set number of iteratios equal to zero
	**mNumIter = 0;

	// store values currently at t=k for later use
	mIdqPrevStep = **mIdq;
	mEdqtsPrevStep = mEdqts;

	// update DQ-DP transforms according to mThetaMech
	mDomainInterface.updateDQToDPTransform(**mThetaMech, mSimTime);
	mDomainInterface.updateDPToDQTransform(**mThetaMech, mSimTime);

	// predict emf at t=k+1 (euler) using
	mEdqts = Math::StateSpaceEuler(mEdqts, mAStateSpace, mBStateSpace, mCStateSpace * **mEf, mTimeStep, **mIdq);

	// predict armature currents for at t=k+1
	(**mIdq)(0,0) = (mEdqts(3,0) - (**mVdq)(1,0) ) / mLd_s;
	(**mIdq)(1,0) = ((**mVdq)(0,0) - mEdqts(2,0) ) / mLq_s;

	// convert currents to dp domain
	(**mIntfCurrent)(0,0) =  mDomainInterface.applyDQToDPTransform(**mIdq) * mBase_I_RMS;
}

void DP::Ph1::SynchronGenerator6OrderPCM::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0,0), (**mIntfCurrent)(0, 0));
}

void DP::Ph1::SynchronGenerator6OrderPCM::correctorStep() {
	// corrector step (trapezoidal rule)
	**mNumIter = **mNumIter + 1;

	// correct emf at t=k+1 (trapezoidal rule)
	mEdqts = Math::applyStateSpaceTrapezoidalMatrices(mAdTrapezoidal, mBdTrapezoidal, mCdTrapezoidal * **mEf, mEdqtsPrevStep, **mIdq, mIdqPrevStep);

	// calculate corrected stator currents at t=k+1 (assuming Vdq(k+1)=VdqPrevIter(k+1))
	(**mIdq)(0,0) = (mEdqts(3,0) - (**mVdq)(1,0) ) / mLd_s;
	(**mIdq)(1,0) = ((**mVdq)(0,0) - mEdqts(2,0) ) / mLq_s;

	// convert corrected currents to dp domain
	(**mIntfCurrent)(0,0) = mDomainInterface.applyDQToDPTransform(**mIdq) * mBase_I_RMS;

	// stamp currents
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::SynchronGenerator6OrderPCM::updateVoltage(const Matrix& leftVector) {
	// store voltage value currently at j-1 for later use
	mVdqPrevIter = **mVdq;

	//
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

	// convert armature voltage into dq reference frame
	**mVdq = mDomainInterface.applyDPToDQTransform((**mIntfVoltage)(0, 0)) / mBase_V_RMS;
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
	**mEdq_t << mEdqts(0,0), mEdqts(1,0);
	**mEdq_s << mEdqts(2,0), mEdqts(3,0);
}

