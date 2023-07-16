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
	mEhMod(mAttributes->create<Complex>("Ehmod")),
	mEdq_t(mAttributes->create<Matrix>("Edq_t"))  {

	mSGOrder = SGOrder::SG4Order;
	mPhaseType = PhaseType::Single;
	setTerminalNumber(1);

	// initialize attributes
	mNumIter = mAttributes->create<Int>("NIterations", 0);

	// model variables
	**mEdq_t = Matrix::Zero(2,1);
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

	SPDLOG_LOGGER_INFO(mSLog,
		"Set base parameters: \n"
		"nomPower: {:e}\nnomVolt: {:e}\nnomFreq: {:e}\n",
		nomPower, nomVolt, nomFreq);

	SPDLOG_LOGGER_INFO(mSLog,
		"Set operational parameters in per unit: \n"
		"inertia: {:e}\n"
		"Ld: {:e}\nLq: {:e}\nL0: {:e}\n"
		"Ld_t: {:e}\nLq_t: {:e}\n"
		"Td0_t: {:e}\nTq0_t: {:e}\n",
		H, Ld, Lq, L0,
		Ld_t, Lq_t,
		Td0_t, Tq0_t);
};

void DP::Ph1::SynchronGenerator4OrderTPM::calculateStateSpaceMatrices() {
	mAStateSpace <<	-mLq / mTq0_t / mLq_t,	0,
              		0,						-mLd / mTd0_t / mLd_t;
	mBStateSpace <<	(mLq-mLq_t) / mTq0_t / mLq_t,	0.0,
					0.0,							(mLd-mLd_t) / mTd0_t / mLd_t;
	mCStateSpace <<	0,
					1 / mTd0_t;
	Math::calculateStateSpaceTrapezoidalMatrices(mAStateSpace, mBStateSpace, mCStateSpace, mTimeStep, mAdStateSpace, mBdStateSpace, mCdStateSpace);
}

void DP::Ph1::SynchronGenerator4OrderTPM::specificInitialization() {
	// initial emf in the dq reference frame
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;
	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Model specific initialization  ---"
		"\nInitial Ed_t (per unit): {:f}"
		"\nInitial Eq_t (per unit): {:f}"
		"\nSG Model: 4 Order TPM"
		"\n--- Model specific initialization finished ---",
		(**mEdq_t)(0,0),
		(**mEdq_t)(1,0)
	);

	calculateStateSpaceMatrices();

	mDomainInterface.setDPShiftFrequency(mBase_OmMech);
}

void DP::Ph1::SynchronGenerator4OrderTPM::calculateConstantConductanceMatrix() {
	Matrix resistanceMatrix = Matrix::Zero(2,2);
	resistanceMatrix(0,0) = 0;
	resistanceMatrix(0,1) = (mA - mB) / 2.0;
	resistanceMatrix(1,0) = - (mA - mB) / 2.0;
	resistanceMatrix(1,1) = 0;

	SPDLOG_LOGGER_INFO(mSLog, "\nR_const [pu]: {}", Logger::matrixToString(resistanceMatrix));

	resistanceMatrix = resistanceMatrix * mBase_Z;
	SPDLOG_LOGGER_INFO(mSLog, "\nR_const [Ohm]: {}", Logger::matrixToString(resistanceMatrix));

	mConductanceMatrixConst = resistanceMatrix.inverse();
	SPDLOG_LOGGER_INFO(mSLog, "\nG_const [S]: {}", Logger::matrixToString(mConductanceMatrixConst));
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	updateMatrixNodeIndices();

	calculateConstantConductanceMatrix();

	if (mModelAsNortonSource) {
		// Stamp conductance matrix
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrixConst);
	} else {
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
}

void DP::Ph1::SynchronGenerator4OrderTPM::stepInPerUnit() {
	// set number of iteratios equal to zero
	**mNumIter = 0;

	// update DP-DQ transforms
	mDomainInterface.updateDQToDPTransform(**mThetaMech, mSimTime);
	mDomainInterface.updateDPToDQTransform(**mThetaMech, mSimTime);

	// update varying resistance matrix part
	Real DeltaTheta = **mThetaMech - mBase_OmMech * mSimTime;
	mResistanceMatrixVarying(0,0) = - (mA + mB) / 2.0 * sin(2*DeltaTheta);
	mResistanceMatrixVarying(0,1) = (mA + mB) / 2.0 * cos(2*DeltaTheta);
	mResistanceMatrixVarying(1,0) = (mA + mB) / 2.0 * cos(2*DeltaTheta);
	mResistanceMatrixVarying(1,1) = (mA + mB) / 2.0 * sin(2*DeltaTheta);
	SPDLOG_LOGGER_DEBUG(mSLog, "\nR_var [pu] (t={:f}): {:s}", mSimTime, Logger::matrixToString(mResistanceMatrixVarying));

	// predict electrical vars
	// set previous values of stator current at simulation start
	if (mSimTime == 0.0) {
		(**mIntfCurrent)(0,0) = std::conj(mInitElecPower / (mInitVoltage * mBase_V_RMS));
		mIdpTwoPrevStep = **mIntfCurrent;
	}

	// predict stator current (linear extrapolation)
	Matrix IdpPrediction = Matrix::Zero(2,1);
	IdpPrediction(0,0) = 2 * (**mIntfCurrent)(0,0).real() - mIdpTwoPrevStep(0,0).real();
	IdpPrediction(1,0) = 2 * (**mIntfCurrent)(0,0).imag() - mIdpTwoPrevStep(0,0).imag();

	// calculate emf at t=k
	(**mEdq_t)(0,0) = (**mVdq)(0,0) - (**mIdq)(1,0) * mLq_t;
	(**mEdq_t)(1,0) = (**mVdq)(1,0) + (**mIdq)(0,0) * mLd_t;

	// calculate original history voltage of VBR model (trapezoidal rule)
	mEh(0,0) = mAd_t * (**mIdq)(1,0) + mBd_t * (**mEdq_t)(0,0);
	mEh(1,0) = mAq_t * (**mIdq)(0,0) + mBq_t * (**mEdq_t)(1,0) + mDq_t * mEf_prev + mDq_t * (**mEf);

	// set to original history voltage in dp domain
	**mEhMod = mDomainInterface.applyDQToDPTransform(mEh) * mBase_V_RMS;

	// add current prediction based component to modified history voltage of TPM model in dp domain
	**mEhMod += - Complex(mBase_Z * (mResistanceMatrixVarying * IdpPrediction)(0,0), 0);
	**mEhMod += - Complex(0, mBase_Z * (mResistanceMatrixVarying * IdpPrediction)(1,0));

	// store values currently at t=k for later use
	mIdpTwoPrevStep = **mIntfCurrent;
	mVdqPrevStep = **mVdq;
	mEdqtPrevStep = **mEdq_t;
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mModelAsNortonSource) {
		// Determine equivalent Norton current
		mIhMod = Complex(mConductanceMatrixConst(0,0) * (**mEhMod).real() + mConductanceMatrixConst(0,1) * (**mEhMod).imag(),
					    mConductanceMatrixConst(1,0) * (**mEhMod).real() + mConductanceMatrixConst(1,1) * (**mEhMod).imag());
		// Stamp Norton current
		Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mIhMod);
	} else {
		// Stamp modified history voltage
		Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(), **mEhMod);
	}
}

void DP::Ph1::SynchronGenerator4OrderTPM::correctorStep() {
	// increase number of iterations
	**mNumIter = **mNumIter + 1;

	// correct electrical vars
	// calculate emf at j and k+1 (trapezoidal rule)
	(**mEdq_t) = Math::applyStateSpaceTrapezoidalMatrices(mAdStateSpace, mBdStateSpace, mCdStateSpace * **mEf, mEdqtPrevStep, **mVdq, mVdqPrevStep);

	// calculate stator currents at j and k+1
	(**mIdq)(0,0) = ((**mEdq_t)(1,0) - (**mVdq)(1,0) ) / mLd_t;
	(**mIdq)(1,0) = ((**mVdq)(0,0) - (**mEdq_t)(0,0) ) / mLq_t;

	// convert corrected currents to dp domain
	Complex IdpCorrectionComplex = mDomainInterface.applyDQToDPTransform(**mIdq) * mBase_I_RMS;

	Matrix IdpCorrection = Matrix::Zero(2,1);
	IdpCorrection(0,0) = IdpCorrectionComplex.real();
	IdpCorrection(1,0) = IdpCorrectionComplex.imag();

	// reset to original history voltage of VBR model in dp domain
	**mEhMod = mDomainInterface.applyDQToDPTransform(mEh) * mBase_V_RMS;

	// add current correction based component to modified history voltage of TPM model in dp domain
	**mEhMod += - Complex(mBase_Z * (mResistanceMatrixVarying * IdpCorrection)(0,0), 0);
	**mEhMod += - Complex(0, mBase_Z * (mResistanceMatrixVarying * IdpCorrection)(1,0));

	// stamp currents
	(**mRightVector).setZero();
	mnaCompApplyRightSideVectorStamp(**mRightVector);

	// store value currently at j-1 for later use
	mVdqPrevIter = **mVdq;
}

void DP::Ph1::SynchronGenerator4OrderTPM::updateVoltage(const Matrix& leftVector) {
	// update armature voltage
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

	// convert armature voltage to dq reference frame
	**mVdq = mDomainInterface.applyDPToDQTransform((**mIntfVoltage)(0, 0)) / mBase_V_RMS;
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
		Matrix voltageDifference = **mVdq - mVdqPrevIter;
		if (Math::abs(voltageDifference(0,0)) > mTolerance || Math::abs(voltageDifference(1,0)) > mTolerance)
			return true;
		else
			return false;
	}
}

void DP::Ph1::SynchronGenerator4OrderTPM::mnaCompPostStep(const Matrix& leftVector) {
	// update armature current
	if (mModelAsNortonSource) {
		(**mIntfCurrent)(0, 0) = mIhMod - Complex(mConductanceMatrixConst(0,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrixConst(0,1) * (**mIntfVoltage)(0, 0).imag(),
					    						 mConductanceMatrixConst(1,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrixConst(1,1) * (**mIntfVoltage)(0, 0).imag());
	} else {
		(**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex());
	}

	// convert armature current to dq reference frame
	**mIdq = mDomainInterface.applyDPToDQTransform((**mIntfCurrent)(0, 0)) / mBase_I_RMS;
}
