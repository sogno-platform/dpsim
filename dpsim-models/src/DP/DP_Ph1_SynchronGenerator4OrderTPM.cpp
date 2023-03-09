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
	mEvbr(mAttributes->create<Complex>("Evbr")),
	mEdq_t(mAttributes->create<Matrix>("Edq"))  {

	mSGOrder = SGOrder::SG4Order;
	mPhaseType = PhaseType::Single;
	setVirtualNodeNumber(2);
	setTerminalNumber(1);

	/// initialize attributes
	mNumIter = mAttributes->create<Int>("NIterations", 0);

	// TODO: Remove using MathUtils
	mShiftVector = Matrix::Zero(3,1);
	mShiftVector << Complex(1., 0), SHIFT_TO_PHASE_B, SHIFT_TO_PHASE_C;

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
	mKvbr = Matrix::Zero(1,2);
	mKvbr(0,0) = Complex(cos(**mThetaMech - mBase_OmMech * mSimTime), sin(**mThetaMech - mBase_OmMech * mSimTime));
	mKvbr(0,1) = -Complex(cos(**mThetaMech - mBase_OmMech * mSimTime - PI/2.), sin(**mThetaMech - mBase_OmMech * mSimTime - PI/2.));
}

void DP::Ph1::SynchronGenerator4OrderTPM::calculateAuxiliarConstants() {
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
	// initial emf in the dq reference frame
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	calculateAuxiliarConstants();

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
	resistanceMatrix(0,0) = 0;
	resistanceMatrix(0,1) = (mA - mB) / 2.0;
	resistanceMatrix(1,0) = - (mA - mB) / 2.0;
	resistanceMatrix(1,1) = 0;

	SPDLOG_LOGGER_INFO(mSLog, "\nR_const [pu]: {}", Logger::matrixToString(resistanceMatrix));

	resistanceMatrix = resistanceMatrix * mBase_Z;
	SPDLOG_LOGGER_INFO(mSLog, "\nR_const [Ohm]: {}", Logger::matrixToString(resistanceMatrix));

	mConductanceMatrixConst = resistanceMatrix.inverse();

	mSLog->info(
		"\n--- Model specific initialization  ---",
		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0)
	);
	mSLog->flush();
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaCompApplySystemMatrixStamp(Matrix& systemMatrix) {

	updateMatrixNodeIndices();
	calculateConductanceMatrix();

	// Stamp voltage source
	Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex(), Complex(-1, 0));
	Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));

	// Stamp conductance
	// set upper left block
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), mConductanceMatrixConst);

	// set buttom right block
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrixConst);

	// Set off diagonal blocks
	Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0), -mConductanceMatrixConst);
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(), -mConductanceMatrixConst);
}

void DP::Ph1::SynchronGenerator4OrderTPM::stepInPerUnit() {
	// set number of iteratios equal to zero
	**mNumIter = 0;

	// update auxiliar variables for time-varying VBR voltage and matrix depending on mThetaMech
	calculateAuxiliarVariables();

	Real DeltaTheta = **mThetaMech - mBase_OmMech * mSimTime;
	mResistanceMatrixVarying(0,0) = - (mA + mB) / 2.0 * sin(2*DeltaTheta);
	mResistanceMatrixVarying(0,1) = (mA + mB) / 2.0 * cos(2*DeltaTheta);
	mResistanceMatrixVarying(1,0) = (mA + mB) / 2.0 * cos(2*DeltaTheta);
	mResistanceMatrixVarying(1,1) = (mA + mB) / 2.0 * sin(2*DeltaTheta);;
	SPDLOG_LOGGER_DEBUG(mSLog, "\nR_var [pu] (t={:f}): {:s}", mSimTime, Logger::matrixToString(mResistanceMatrixVarying));

	// predict electrical vars
	// set previous values of stator current at simulation start
	if (mSimTime == 0.0) {
		(**mIntfCurrent)(0,0) = std::conj(mInitElecPower / (mInitVoltage * mBase_V_RMS));
		mIdpTwoPrev = **mIntfCurrent;
	}

	// predict stator current (linear extrapolation)
	Matrix IdpPrediction = Matrix::Zero(2,1);
	IdpPrediction(0,0) = 2 * (**mIntfCurrent)(0,0).real() - mIdpTwoPrev(0,0).real();
	IdpPrediction(1,0) = 2 * (**mIntfCurrent)(0,0).imag() - mIdpTwoPrev(0,0).imag();

	// calculate emf at t=k
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	// calculate original VBR voltage (trapezoidal rule)
	mEh_vbr(0,0) = mAd_t * (**mIdq)(1,0) + mBd_t * (**mEdq_t)(0,0);
	mEh_vbr(1,0) = mAq_t * (**mIdq)(0,0) + mBq_t * (**mEdq_t)(1,0) + mDq_t * mEf_prev + mDq_t * (**mEf);

	// convert original VBR voltage to dp domain
	**mEvbr = (mKvbr * mEh_vbr * mBase_V_RMS)(0,0);

	// add current prediction based component to VBR voltage in dp domain
	**mEvbr += - Complex(mBase_Z * (mResistanceMatrixVarying * IdpPrediction)(0,0), 0);
	**mEvbr += - Complex(0, mBase_Z * (mResistanceMatrixVarying * IdpPrediction)(1,0));

	// Store previous current for later use
	mIdpTwoPrev = **mIntfCurrent;
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(), **mEvbr);
}

void DP::Ph1::SynchronGenerator4OrderTPM::correctorStep() {
	// increase number of iterations
	**mNumIter = **mNumIter + 1;

	// correct electrical vars
	// CHECK: Is this computation really required? PCM models does
	// simply mEdq_pred = mEdq_corr at the end of each iteration
	mEdq_pred(0,0) = (**mVdq)(0,0) - mIdq_pred(1,0) * mLq_t;
	mEdq_pred(1,0) = (**mVdq)(1,0) + mIdq_pred(0,0) * mLd_t;

	// correct emf at t=k+1 (trapezoidal rule)
	mEdq_corr = mA_prev * **mEdq_t + mA_corr * mEdq_pred + mB_corr * (**mIdq + mIdq_pred) + mC_corr * (**mEf);

	// calculate corrected stator currents at t=k+1 (assuming Vdq(k+1)=VdqPrevIter(k+1))
	mIdq_corr(0,0) = (mEdq_corr(1,0) - (**mVdq)(1,0) ) / mLd_t;
	mIdq_corr(1,0) = ((**mVdq)(0,0) - mEdq_corr(0,0) ) / mLq_t;

	// convert corrected currents to dp domain
	Complex IdpCorrectionComplex = (mKvbr * mIdq_corr)(0,0) * mBase_I_RMS;

	Matrix IdpCorrection = Matrix::Zero(2,1);
	IdpCorrection(0,0) = IdpCorrectionComplex.real();
	IdpCorrection(1,0) = IdpCorrectionComplex.imag();

	// reset original VBR voltage in dp domain
	**mEvbr = (mKvbr * mEh_vbr * mBase_V_RMS)(0,0);

	// add current correction based component to VBR voltage in dp domain
	**mEvbr += - Complex(mBase_Z * (mResistanceMatrixVarying * IdpCorrection)(0,0), 0);
	**mEvbr += - Complex(0, mBase_Z * (mResistanceMatrixVarying * IdpCorrection)(1,0));

	// stamp currents
	(**mRightVector).setZero();
	mnaCompApplyRightSideVectorStamp(**mRightVector);
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

bool DP::Ph1::SynchronGenerator4OrderTPM::requiresIteration() {
	if (**mNumIter >= mMaxIter) {
		// maximum number of iterations reached
		return false;
	} else if (**mNumIter == 0) {
		// no corrector step has been performed yet,
		// convergence cannot be confirmed
		return true;
	} else {
		// check voltage convergence according to tolerance
		Matrix voltageDifference = **mVdq - mVdq_prev;
		if (Math::abs(voltageDifference(0,0)) > mTolerance || Math::abs(voltageDifference(1,0)) > mTolerance)
			return true;
		else
			return false;
	}
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaCompPostStep(const Matrix& leftVector) {
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
