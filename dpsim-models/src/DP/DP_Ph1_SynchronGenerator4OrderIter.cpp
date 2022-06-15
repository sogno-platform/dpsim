/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_SynchronGenerator4OrderIter.h>

using namespace CPS;

DP::Ph1::SynchronGenerator4OrderIter::SynchronGenerator4OrderIter
    (String uid, String name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel) {

	mPhaseType = PhaseType::Single;
	setTerminalNumber(1);

	// model flags
	mVoltageForm = false;
	
	// model variables
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
	mEdq_t = Matrix::Zero(2,1);
	mEdq_t_pred = Matrix::Zero(2,1);
	mEdq_t_corr = Matrix::Zero(2,1);
	mdEdq_t = Matrix::Zero(2,1);
	mdEdq_t_corr = Matrix::Zero(2,1);

	//
	mShiftVector = Matrix::Zero(3,1);
	mShiftVector << Complex(1., 0), SHIFT_TO_PHASE_B, SHIFT_TO_PHASE_C;

    // Register attributes
	addAttribute<Int>("NIterations", &mNumIterations2 , Flags::read);
	addAttribute<Matrix>("Edq0_t", &mEdq_t, Flags::read);
}

DP::Ph1::SynchronGenerator4OrderIter::SynchronGenerator4OrderIter
	(String name, Logger::Level logLevel)
	: SynchronGenerator4OrderIter(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator4OrderIter::clone(String name) {
	auto copy = SynchronGenerator4OrderIter::make(name, mLogLevel);
	
	return copy;
}

void DP::Ph1::SynchronGenerator4OrderIter::specificInitialization() {

	// Initialize matrix of state representation
	mA = Matrix::Zero(2,2);
	mB = Matrix::Zero(2,2);
	mC = Matrix::Zero(2,1);
	calculateStateMatrix();

	// initial voltage behind the transient reactance in the dq0 reference frame
	mEdq_t(0,0) = mVdq(0,0) - mIdq(1,0) * mLq_t;
	mEdq_t(1,0) = mVdq(1,0) + mIdq(0,0) * mLd_t;

	// initialize transformation matrix dp->dq
	mDpToDq = Matrix::Zero(1,2);

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nMax number of iterations: {:d}"
		"\nTolerance: {:f}"
		"\n--- Model specific initialization finished ---",

		mEdq_t(0,0),
		mEdq_t(1,0),
		mMaxIter,
		mTolerance
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator4OrderIter::calculateStateMatrix() {
	if (mVoltageForm) {
		Real Td_t =  mTd0_t * (mLd_t / mLd);
		Real Tq_t =  mTq0_t * (mLq_t / mLq);
		mA << -1. / Tq_t   ,          0.0,
	               0.0     ,      -1 / Td_t;
		mB << (1. / Tq_t) * (mLq-mLq_t) / mLq,   				0.0, 				
		  					0.0, 					(1. / Td_t) * (mLd-mLd_t) / mLd;
		mC <<               0.0,
		   		(1. / Td_t) * mEf * (mLd_t / mLd);
	}
	else {	// Currents form
		mA << -1./mTq0_t	,    	0.0,			
		     	   0.0   	,    -1/mTd0_t;
		mB <<             0.0                  , (1. / mTq0_t) * (mLq-mLq_t),
		  		(-1. / mTd0_t) * (mLd-mLd_t)   ,            0.0             ;
		mC <<          0.0,
		 		 (1./mTd0_t) * mEf;
	}
}

void DP::Ph1::SynchronGenerator4OrderIter::stepInPerUnit() {
	// set number of iteratios equal to zero
	mNumIter = 0;

	// Predictor step (euler)

	//predict mechanical variables at t=k+1
	if (mSimTime>0.0) {
		mElecTorque = mVdq(0,0) * mIdq(0,0) + mVdq(1,0) * mIdq(1,0);
		mdOmMech = 1 / (2.* mH) * (mMechTorque - mElecTorque);
		mOmMech_pred = mOmMech + mTimeStep * mdOmMech;
		mdDelta = (mOmMech_pred - 1.) * mBase_OmMech;
		mDelta_pred = mDelta + mTimeStep * mdDelta;
		mThetaMech_pred = mThetaMech + mTimeStep * mOmMech_pred * mBase_OmMech;
	} else {
		mdOmMech = 0;
		mOmMech_pred = mOmMech;
		mdDelta = 0;
		mDelta_pred = mDelta;
		mThetaMech_pred = mThetaMech;
	}

	//predict voltage behind transient reactance 
	if (mVoltageForm)
		mdEdq_t = mA * mEdq_t + mB * mVdq + mC;
	else
		mdEdq_t = mA * mEdq_t + mB * mIdq + mC;
	mEdq_t_pred = mEdq_t + mTimeStep * mdEdq_t;

	// predict armature currents for at t=k+1
	mIdq(0,0) = (mEdq_t_pred(1,0) - mVdq(1,0) ) / mLd_t;
	mIdq(1,0) = (mVdq(0,0) - mEdq_t_pred(0,0) ) / mLq_t;

	// convert currents into the abc reference frame
	mDpToDq(0,0) = Complex(cos(mThetaMech_pred - mBase_OmMech * mSimTime), sin(mThetaMech_pred - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(mThetaMech_pred - mBase_OmMech * mSimTime - PI/2.), sin(mThetaMech_pred - mBase_OmMech * mSimTime - PI/2.));
	mIntfCurrent(0,0) = (mDpToDq * mIdq)(0,0) * mBase_I_RMS;
}

void DP::Ph1::SynchronGenerator4OrderIter::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mIntfCurrent(0, 0));
}

void DP::Ph1::SynchronGenerator4OrderIter::correctorStep() {
	// corrector step (trapezoidal rule)

	if (mNumIter==0)
		return;

	//predict mechanical variables
	if (mSimTime>0.0) {
		mElecTorque_corr = mVdq(0,0) * mIdq(0,0) + mVdq(1,0) * mIdq(1,0);
		mdOmMech_corr = 1 / (2.* mH) * (mMechTorque - mElecTorque_corr);
		mOmMech_corr = mOmMech + mTimeStep / 2. * (mdOmMech + mdOmMech_corr);
		mdDelta_corr = (mOmMech_corr - 1.) * mBase_OmMech;
		mDelta_corr = mDelta + mTimeStep / 2. * (mdDelta + mdDelta_corr);
		mThetaMech_corr = mThetaMech + mTimeStep / 2. *(mOmMech + mOmMech_corr) * mBase_OmMech;
	} else {
		mElecTorque_corr = mElecTorque;
		mdOmMech_corr = 0;
		mOmMech_corr = mOmMech;
		mdDelta_corr = 0;
		mDelta_corr = mDelta;
		mThetaMech_corr = mThetaMech;
	}

	//predict voltage behind transient reactance
	if (mVoltageForm)
		mdEdq_t_corr = mA * mEdq_t + mB * mVdq + mC;
	else
		mdEdq_t_corr = mA * mEdq_t + mB * mIdq + mC;
	mEdq_t_corr = mEdq_t + mTimeStep / 2 * (mdEdq_t + mdEdq_t_corr);

	// armature currents for at t=k+1
	mIdq(0,0) = (mEdq_t_corr(1,0) - mVdq(1,0) ) / mLd_t;
	mIdq(1,0) = (mVdq(0,0) - mEdq_t_corr(0,0) ) / mLq_t;

	// convert currents into the abc reference frame
	mDpToDq(0,0) = Complex(cos(mThetaMech_corr - mBase_OmMech * mSimTime), sin(mThetaMech_corr - mBase_OmMech * mSimTime));
	mDpToDq(0,1) = -Complex(cos(mThetaMech_corr - mBase_OmMech * mSimTime - PI/2.), sin(mThetaMech_corr - mBase_OmMech * mSimTime - PI/2.));
	mIntfCurrent(0,0) = (mDpToDq * mIdq)(0,0) * mBase_I_RMS;

	// stamp currents
	mnaApplyRightSideVectorStamp(mRightVector);
}

void DP::Ph1::SynchronGenerator4OrderIter::updateVoltage(const Matrix& leftVector) {
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

bool DP::Ph1::SynchronGenerator4OrderIter::checkVoltageDifference() {
	if (mNumIter==0) {
		// if no corrector step has been performed yet
		mNumIter = 1;
		return true;
	}

	Matrix voltageDifference = mVdq - mVdq_prev;
	if (Math::abs(voltageDifference(0,0)) > mTolerance || Math::abs(voltageDifference(1,0)) > mTolerance) {
		if (mNumIter > mMaxIter) {
			return false;
		} else {
			mNumIter = mNumIter + 1;
			return true;
		}
	} else {
		return false;
	}
	
}

void DP::Ph1::SynchronGenerator4OrderIter::mnaPostStep(const Matrix& leftVector) {
	// update variables
	mEdq_t = mEdq_t_corr;
	mOmMech = mOmMech_corr;
	mThetaMech = mThetaMech_corr;
	mDelta = mDelta_corr;

	//
	mNumIterations2 = mNumIter;
}

Matrix DP::Ph1::SynchronGenerator4OrderIter::parkTransform(Real theta, const Matrix& abcVector) {
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