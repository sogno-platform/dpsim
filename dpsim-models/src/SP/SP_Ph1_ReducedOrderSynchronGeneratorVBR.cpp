/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_ReducedOrderSynchronGeneratorVBR.h>

using namespace CPS;

SP::Ph1::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR
    (const String & uid, const String & name, Logger::Level logLevel)
	: Base::ReducedOrderSynchronGenerator<Complex>(uid, name, logLevel) {

	mPhaseType = PhaseType::Single;
	setTerminalNumber(1);

	// model variables
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
}

SP::Ph1::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR
	(const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(name, name, logLevel) {
}

void SP::Ph1::ReducedOrderSynchronGeneratorVBR::initializeResistanceMatrix() {
	// resistance matrix in dq reference frame
	mResistanceMatrixDq = Matrix::Zero(2,2);
	mResistanceMatrixDq <<	0.0,	mA,
					  		mB,		0.0;

	// initialize conductance matrix
	mConductanceMatrix = Matrix::Zero(2,2);
}

void SP::Ph1::ReducedOrderSynchronGeneratorVBR::calculateResistanceMatrix() {
	Matrix resistanceMatrix =  mDqToComplexA * mResistanceMatrixDq * mComplexAToDq;
	resistanceMatrix = resistanceMatrix * mBase_Z;
	mConductanceMatrix = resistanceMatrix.inverse();
}

void SP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompInitialize(Real omega,
		Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	Base::ReducedOrderSynchronGenerator<Complex>::mnaCompInitialize(omega, timeStep, leftVector);

	if (mModelAsNortonSource) {
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

	SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
	for (auto indexPair : mVariableSystemMatrixEntries)
		SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);
}

void SP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {

	if (mModelAsNortonSource) {
		// Stamp conductance matrix
		// set buttom right block
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrix);
	}
	else {
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

void SP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mModelAsNortonSource) {
		// compute equivalent northon circuit in abc reference frame
		mIvbr = Complex(mConductanceMatrix(0,0) * mEvbr.real() + mConductanceMatrix(0,1) * mEvbr.imag(),
					    mConductanceMatrix(1,0) * mEvbr.real() + mConductanceMatrix(1,1) * mEvbr.imag());

		Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mIvbr);
	}
	else {
		Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(), mEvbr);
	}
}

void SP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompPostStep(const Matrix& leftVector) {
	// update armature voltage
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

	// convert armature voltage into dq reference frame
	Matrix Vabc = Matrix::Zero(2,1);
	Vabc << (**mIntfVoltage)(0, 0).real(), (**mIntfVoltage)(0, 0).imag();
	**mVdq = mComplexAToDq * Vabc / mBase_V_RMS;

	// update armature current
	if (mModelAsNortonSource) {
		(**mIntfCurrent)(0, 0) = mIvbr - Complex(mConductanceMatrix(0,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrix(0,1) * (**mIntfVoltage)(0, 0).imag(),
					    						 mConductanceMatrix(1,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrix(1,1) * (**mIntfVoltage)(0, 0).imag());
	} else {
		(**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex());
	}

	// convert armature current into dq reference frame
	Matrix Iabc = Matrix::Zero(2,1);
	Iabc << (**mIntfCurrent)(0, 0).real(), (**mIntfCurrent)(0, 0).imag();
	**mIdq = mComplexAToDq * Iabc / mBase_I_RMS;

}

Matrix SP::Ph1::ReducedOrderSynchronGeneratorVBR::get_DqToComplexATransformMatrix() const {
	Matrix dqToComplexA(2, 2);
	dqToComplexA <<
		cos(**mThetaMech - mBase_OmMech * mSimTime),	-sin(**mThetaMech - mBase_OmMech * mSimTime),
		sin(**mThetaMech - mBase_OmMech * mSimTime),	cos(**mThetaMech - mBase_OmMech * mSimTime);

	return dqToComplexA;
}
