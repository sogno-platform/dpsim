/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_ReducedOrderSynchronGeneratorVBR.h>

using namespace CPS;

DP::Ph1::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR
    (const String & uid, const String & name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel) {

	mPhaseType = PhaseType::Single;
	setTerminalNumber(1);
	
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
}

DP::Ph1::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR
	(const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(name, name, logLevel) {
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::initializeResistanceMatrix() {
	// constant part of ABC resistance matrix
	mResistanceMatrix_const = Matrix::Zero(1,3);
	mResistanceMatrix_const <<	-mL0,	-sqrt(3) / 2. * (mA - mB) - mL0,	sqrt(3) / 2. * (mA - mB) - mL0;
	mResistanceMatrix_const = (-1. / 3.) * mResistanceMatrix_const;
	mR_const_1ph = (mResistanceMatrix_const * mShiftVector)(0,0);

	//
	mKc = Matrix::Zero(1,3);
	mKc << Complex(cos(PI/2.), -sin(PI/2.)), Complex(cos(7.*PI/6.), -sin(7.*PI/6.)), Complex(cos(PI/6.), sin(PI/6.));
	mKc = (-1. / 6.) * (mA + mB) * mKc;
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::calculateConductanceMatrix() {
	Matrix resistanceMatrix = Matrix::Zero(2,2);
	resistanceMatrix(0,0) = mR_const_1ph.real() + mKa_1ph.real() + mKb_1ph.real();
	resistanceMatrix(0,1) = -mR_const_1ph.imag() - mKa_1ph.imag() + mKb_1ph.imag();
	resistanceMatrix(1,0) = mR_const_1ph.imag() + mKa_1ph.imag() + mKb_1ph.imag();
	resistanceMatrix(1,1) = mR_const_1ph.real() + mKa_1ph.real() - mKb_1ph.real();
	resistanceMatrix = resistanceMatrix * mBase_Z;
	mConductanceMatrix = resistanceMatrix.inverse();
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::calculateAuxiliarVariables() {	
	mKa = Matrix::Zero(1,3);	
	mKa = mKc * Complex(cos(2. * **mThetaMech), sin(2. * **mThetaMech));
	mKa_1ph = (mKa * mShiftVector)(0,0);

	mKb = Matrix::Zero(1,3);	
	Real arg = 2. * **mThetaMech - 2. * mBase_OmMech * mSimTime ;
	mKb = mKc * Complex(cos(arg), sin(arg));
	mKb_1ph = (mKb * mShiftVectorConj)(0,0);

	mKvbr = Matrix::Zero(1,2);
	mKvbr(0,0) = Complex(cos(**mThetaMech - mBase_OmMech * mSimTime), sin(**mThetaMech - mBase_OmMech * mSimTime));
	mKvbr(0,1) = -Complex(cos(**mThetaMech - mBase_OmMech * mSimTime - PI/2.), sin(**mThetaMech - mBase_OmMech * mSimTime - PI/2.));
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaInitialize(Real omega, 
		Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	Base::ReducedOrderSynchronGenerator<Complex>::mnaInitialize(omega, timeStep, leftVector);

	if (mModelAsCurrentSource) {
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), matrixNodeIndex(0, 0)));
	} else {
		// upper left
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex()));

		// buttom right
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), matrixNodeIndex(0, 0)));

		// off diagonal
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0)));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex()));
	}
	
	mSLog->info("List of index pairs of varying matrix entries: ");
	for (auto indexPair : mVariableSystemMatrixEntries)
		mSLog->info("({}, {})", indexPair.first, indexPair.second);
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (mModelAsCurrentSource) {
		// Stamp conductance matrix
		// set buttom right block
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrix);
	
	} else {
		// Stamp voltage source
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex(), Complex(-1, 0));
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));

		// Stamp conductance matrix

		// set upper left block
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), mConductanceMatrix);

		// set buttom right block
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrix);

		// Set off diagonal blocks
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0), -mConductanceMatrix);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(), -mConductanceMatrix);
	}
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mModelAsCurrentSource) {
		// compute equivalent northon circuit in abc reference frame
		mIvbr = Complex(mConductanceMatrix(0,0) * mEvbr.real() + mConductanceMatrix(0,1) * mEvbr.imag(), 
					    mConductanceMatrix(1,0) * mEvbr.real() + mConductanceMatrix(1,1) * mEvbr.imag());
		
		Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mIvbr);
	
	} else {
		Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(), mEvbr);
	}
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaPostStep(const Matrix& leftVector) {
	// update armature voltage
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

	// convert armature voltage into dq reference frame
	Matrix parkTransform = get_parkTransformMatrix();
	MatrixComp Vabc_ = (**mIntfVoltage)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	auto Vabc = Matrix(3,1);
	Vabc << Vabc_(0,0).real(), Vabc_(1,0).real(), Vabc_(2,0).real();
	**mVdq = parkTransform * Vabc / mBase_V_RMS;

	// update armature current
	if (mModelAsCurrentSource) {
		(**mIntfCurrent)(0, 0) = mIvbr - Complex(mConductanceMatrix(0,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrix(0,1) * (**mIntfVoltage)(0, 0).imag(), 
					    						 mConductanceMatrix(1,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrix(1,1) * (**mIntfVoltage)(0, 0).imag());
	} else {
		(**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex());
	}

	// convert armature current into dq reference frame
	MatrixComp Iabc_ = (**mIntfCurrent)(0, 0) * mShiftVector * Complex(cos(mNomOmega * mSimTime), sin(mNomOmega * mSimTime));
	auto Iabc = Matrix(3,1);
	Iabc << Iabc_(0,0).real(), Iabc_(1,0).real(), Iabc_(2,0).real();
	**mIdq = parkTransform * Iabc / mBase_I_RMS;
}

Matrix DP::Ph1::ReducedOrderSynchronGeneratorVBR::get_parkTransformMatrix() const {
	Matrix abcToDq0(2, 3);

	abcToDq0 <<
		2./3.*cos(**mThetaMech),  2./3.*cos(**mThetaMech - 2.*PI/3.),  2./3.*cos(**mThetaMech + 2.*PI/3.),
		-2./3.*sin(**mThetaMech), -2./3.*sin(**mThetaMech - 2.*PI/3.), -2./3.*sin(**mThetaMech + 2.*PI/3.);
	
	return abcToDq0;
}