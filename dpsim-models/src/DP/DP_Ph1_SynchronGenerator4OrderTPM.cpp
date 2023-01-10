/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGenerator4OrderTPM.h>

using namespace CPS;

DP::Ph1::SynchronGenerator4OrderTPM::SynchronGenerator4OrderTPM
    (String uid, String name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel),
	mEvbr(Attribute<Complex>::create("Evbr", mAttributes)),
	mEdq_t(Attribute<Matrix>::create("Edq", mAttributes))  {
	
	mPhaseType = PhaseType::Single;
	setVirtualNodeNumber(2);
	setTerminalNumber(1);

	/// initialize attributes
	mNumIter = Attribute<Int>::create("NIterations", mAttributes, 0);

	// model variables
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);

	// initialize conductance Matrix
    mConductanceMatrix = Matrix::Zero(2,2);

	//
	mShiftVector = Matrix::Zero(3,1);
	mShiftVector << Complex(1., 0), SHIFT_TO_PHASE_B, SHIFT_TO_PHASE_C;
	mShiftVectorConj = Matrix::Zero(3,1);
	mShiftVectorConj << Complex(1., 0), std::conj(SHIFT_TO_PHASE_B), std::conj(SHIFT_TO_PHASE_C);

	// model variables
	mEh_vbr = Matrix::Zero(2,1);
	**mEdq_t = Matrix::Zero(2,1);
	mEdq_corr = Matrix::Zero(2,1);
	mEdq_pred = Matrix::Zero(2,1);
}

DP::Ph1::SynchronGenerator4OrderTPM::SynchronGenerator4OrderTPM
	(String name, Logger::Level logLevel)
	: SynchronGenerator4OrderTPM(name, name, logLevel) {
}

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGenerator4OrderTPM::clone(String name) {
	auto copy = SynchronGenerator4OrderTPM::make(name, mLogLevel);
	
	return copy;
}

void DP::Ph1::SynchronGenerator4OrderTPM::setOperationalParametersPerUnit(Real nomPower, 
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t) {

	Base::ReducedOrderSynchronGenerator<Complex>::setOperationalParametersPerUnit(nomPower, 
			nomVolt, nomFreq, H, Ld, Lq, L0,
			Ld_t, Lq_t, Td0_t, Tq0_t);
	
	mSLog->info("Set base parameters: \n"
				"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n",
				nomPower, nomVolt, nomFreq);

	mSLog->info("Set operational parameters in per unit: \n"
			"inertia: {:e}\n"
			"Ld: {:e}\nLq: {:e}\nL0: {:e}\n"
			"Ld_t: {:e}\nLq_t: {:e}\n"
			"Td0_t: {:e}\nTq0_t: {:e}\n",
			H, Ld, Lq, L0, 
			Ld_t, Lq_t,
			Td0_t, Tq0_t);
};

void DP::Ph1::SynchronGenerator4OrderTPM::calculateAuxiliarVariables() {	
	mKa = Matrix::Zero(1,3);	
	mKa = mKc * Complex(cos(2. * **mThetaMech), sin(2. * **mThetaMech));
	mKa_1ph = (mKa * mShiftVector)(0,0);

	mKb = Matrix::Zero(1,3);	
	Real arg = 2. * **mThetaMech - 2. * mBase_OmMech * mSimTime;
	mKb = mKc * Complex(cos(arg), sin(arg));
	mKb_1ph = (mKb * mShiftVectorConj)(0,0);

	mKvbr = Matrix::Zero(1,2);
	mKvbr(0,0) = Complex(cos(**mThetaMech - mBase_OmMech * mSimTime), sin(**mThetaMech - mBase_OmMech * mSimTime));
	mKvbr(0,1) = -Complex(cos(**mThetaMech - mBase_OmMech * mSimTime - PI/2.), sin(**mThetaMech - mBase_OmMech * mSimTime - PI/2.));
}

void DP::Ph1::SynchronGenerator4OrderTPM::calculateAuxiliarConstants() {
	mAd = mTimeStep * (mLq - mLq_t) / (2 * mTq0_t + mTimeStep);
	mBd = (2 * mTq0_t - mTimeStep) / (2 * mTq0_t + mTimeStep);

	mAq = - mTimeStep * (mLd - mLd_t) / (2 * mTd0_t + mTimeStep);
	mBq = (2 * mTd0_t - mTimeStep) / (2 * mTd0_t + mTimeStep);
	mCq = 2 * mTimeStep * mEf / (2 * mTd0_t + mTimeStep);

	mB = mLd_t - mAq;
	mA = -mLq_t - mAd;

	mKc = Matrix::Zero(1,3);
	mKc << Complex(cos(PI/2.), -sin(PI/2.)), Complex(cos(7.*PI/6.), -sin(7.*PI/6.)), Complex(cos(PI/6.), sin(PI/6.));
	mKc = (-1. / 6.) * (mA + mB) * mKc;

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

void DP::Ph1::SynchronGenerator4OrderTPM::specificInitialization() {
	// initial voltage behind the transient reactance in the dq reference frame
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;
	calculateAuxiliarConstants();

	// constant part of ABC resistance matrix
	mResistanceMatrix_const = Matrix::Zero(1,3);
	mResistanceMatrix_const <<	-mL0,	-sqrt(3) / 2. * (mA - mB) - mL0,	sqrt(3) / 2. * (mA - mB) - mL0;
	mResistanceMatrix_const = (-1. / 3.) * mResistanceMatrix_const;
	mR_const_1ph = (mResistanceMatrix_const * mShiftVector)(0,0);

	mSLog->info(
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\n--- Model specific initialization finished ---",
		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0)
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator4OrderTPM::calculateConductanceMatrix() {
	Matrix resistanceMatrix = Matrix::Zero(2,2);

	resistanceMatrix(0,0) = mR_const_1ph.real();
	resistanceMatrix(0,1) = -mR_const_1ph.imag();
	resistanceMatrix(1,0) = mR_const_1ph.imag();
	resistanceMatrix(1,1) = mR_const_1ph.real();

	resistanceMatrix = resistanceMatrix * mBase_Z;
	mConductanceMatrix = resistanceMatrix.inverse();
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	updateMatrixNodeIndices();
	calculateConductanceMatrix();
	// Stamp voltage source
	Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex(), Complex(-1, 0));
	Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));

	// Stamp conductance

	// set upper left block
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), mConductanceMatrix);

	// set buttom right block
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrix);

	// Set off diagonal blocks
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0), -mConductanceMatrix);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(), -mConductanceMatrix);
}

void DP::Ph1::SynchronGenerator4OrderTPM::stepInPerUnit() {
	// set number of iteratios equal to zero
	**mNumIter = 0;

	// calculate Edq_t at t=k
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	if (mSimTime>0.0){
		// 	calculate mechanical variables at t=k+1 using forward euler
		**mElecTorque = (**mVdq)(0,0) * (**mIdq)(0,0) + (**mVdq)(1,0) * (**mIdq)(1,0);
		**mOmMech = **mOmMech + mTimeStep * (1. / (2. * mH) * (**mMechTorque - **mElecTorque));
		**mThetaMech = **mThetaMech + mTimeStep * (**mOmMech * mBase_OmMech);
		**mDelta = **mDelta + mTimeStep * (**mOmMech - 1.) * mBase_OmMech;
	}

	// VBR history voltage
	calculateAuxiliarVariables();
	//mEh_vbr(0,0) = mAd * (**mIdq)(1,0) + mBd * (**mEdq_t)(0,0);
	//mEh_vbr(1,0) = mAq * (**mIdq)(0,0) + mBq * (**mEdq_t)(1,0) + mCq;

	// predict current
	if (mSimTime==0.0) {
		(**mIntfCurrent)(0,0) = std::conj(mInitElecPower / (mInitVoltage * mBase_V_RMS));
		mIdq_2prev = **mIntfCurrent;
	}
	
	Matrix Idq_pred = Matrix::Zero(2,1);
	Idq_pred(0,0) = 2 * (**mIntfCurrent)(0,0).real() - mIdq_2prev(0,0).real();
	Idq_pred(1,0) = 2 * (**mIntfCurrent)(0,0).imag() - mIdq_2prev(0,0).imag();

	// 
	Matrix resistanceMatrix = Matrix::Zero(2,2);
	resistanceMatrix(0,0) = mKa_1ph.real() + mKb_1ph.real();
	resistanceMatrix(0,1) = - mKa_1ph.imag() + mKb_1ph.imag();
	resistanceMatrix(1,0) = mKa_1ph.imag() + mKb_1ph.imag();
	resistanceMatrix(1,1) = mKa_1ph.real() - mKb_1ph.real();

	mEh_vbr(0,0) = mAd * (**mIdq)(1,0) + mBd * (**mEdq_t)(0,0);
	mEh_vbr(1,0) = mAq * (**mIdq)(0,0) + mBq * (**mEdq_t)(1,0) + mCq;

	// convert Edq_t into the abc reference frame
	**mEvbr = (mKvbr * mEh_vbr * mBase_V_RMS)(0,0);

	//
	**mEvbr += - Complex(mBase_Z * (resistanceMatrix * Idq_pred)(0,0), 0);
	**mEvbr += - Complex(0, mBase_Z * (resistanceMatrix * Idq_pred)(1,0));

	//
	mIdq_2prev = **mIntfCurrent;
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(), **mEvbr);
}

void DP::Ph1::SynchronGenerator4OrderTPM::correctorStep() {
	// corrector step (trapezoidal rule)
	**mNumIter = **mNumIter + 1;

	/*
	// calculate Edq_t corrected
	mEdq_corr(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	mEdq_corr(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	// corrected currents at t=k+1
	mIdq_corr(0,0) = (mEdq_corr(1,0) - (**mVdq)(1,0) ) / mLd_t;
	mIdq_corr(1,0) = ((**mVdq)(0,0) - mEdq_corr(0,0) ) / mLq_t;
	*/

	//predict voltage behind transient reactance (with trapezoidal rule)
	mEdq_pred(0,0) = (**mVdq)(0,0) - mIdq_pred(1,0) * mLq_t;
	mEdq_pred(1,0) = (**mVdq)(1,0) + mIdq_pred(0,0) * mLd_t;
	mEdq_corr = mA_prev * **mEdq_t + mA_corr * mEdq_pred + mB_corr * (**mIdq + mIdq_pred) + mC_corr * mEf;

	// armature currents for at t=k+1
	mIdq_corr(0,0) = (mEdq_corr(1,0) - (**mVdq)(1,0) ) / mLd_t;
	mIdq_corr(1,0) = ((**mVdq)(0,0) - mEdq_corr(0,0) ) / mLq_t;

	// convert currents into the abc reference frame
	Complex Idqpred = (mKvbr * mIdq_corr)(0,0) * mBase_I_RMS;
	Matrix Idq_pred = Matrix::Zero(2,1);
	Idq_pred(0,0) = Idqpred.real();
	Idq_pred(1,0) = Idqpred.imag();
	
	// 
	Matrix resistanceMatrix = Matrix::Zero(2,2);
	resistanceMatrix(0,0) = mKa_1ph.real() + mKb_1ph.real();
	resistanceMatrix(0,1) = - mKa_1ph.imag() + mKb_1ph.imag();
	resistanceMatrix(1,0) = mKa_1ph.imag() + mKb_1ph.imag();
	resistanceMatrix(1,1) = mKa_1ph.real() - mKb_1ph.real();

	// convert Edq_t into the abc reference frame
	**mEvbr = (mKvbr * mEh_vbr * mBase_V_RMS)(0,0);

	//
	**mEvbr += - Complex(mBase_Z * (resistanceMatrix * Idq_pred)(0,0), 0);
	**mEvbr += - Complex(0, mBase_Z * (resistanceMatrix * Idq_pred)(1,0));

	// stamp currents
	(**mRightVector).setZero();
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::SynchronGenerator4OrderTPM::updateVoltage(const Matrix& leftVector) {
	mVdq_prev = **mVdq;
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	(**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex());
	
	//
	Matrix parkTransform = get_parkTransformMatrix();

	// convert armature voltage into dq reference frame
	MatrixComp Vabc_ = (**mIntfVoltage)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	**mVdq = parkTransform * Vabc / mBase_V_RMS;

	// convert armature current into dq reference frame
	MatrixComp Iabc_ = (**mIntfCurrent)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Iabc = Matrix(3,1);
	Iabc << Iabc_(0,0).real(), Iabc_(1,0).real(), Iabc_(2,0).real();
	mIdq_pred = parkTransform * Iabc / mBase_I_RMS;
}

bool DP::Ph1::SynchronGenerator4OrderTPM::checkVoltageDifference() {
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

void DP::Ph1::SynchronGenerator4OrderTPM::mnaPostStep(const Matrix& leftVector) {
	// update armature voltage and current
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	(**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex());

	// convert armature voltage into dq reference frame
	Matrix parkTransform = get_parkTransformMatrix();
	MatrixComp Vabc_ = (**mIntfVoltage)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	**mVdq = parkTransform * Vabc / mBase_V_RMS;

	// convert armature current into dq reference frame
	MatrixComp Iabc_ = (**mIntfCurrent)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	Matrix Iabc = Matrix(3,1);
	Iabc << Iabc_(0,0).real(), Iabc_(1,0).real(), Iabc_(2,0).real();
	**mIdq = parkTransform * Iabc / mBase_I_RMS;
}

Matrix DP::Ph1::SynchronGenerator4OrderTPM::get_parkTransformMatrix() {
	Matrix abcToDq0(2, 3);

	abcToDq0 <<
		2./3.*cos(**mThetaMech),  2./3.*cos(**mThetaMech - 2.*PI/3.),  2./3.*cos(**mThetaMech + 2.*PI/3.),
		-2./3.*sin(**mThetaMech), -2./3.*sin(**mThetaMech - 2.*PI/3.), -2./3.*sin(**mThetaMech + 2.*PI/3.);

	return abcToDq0;
}
