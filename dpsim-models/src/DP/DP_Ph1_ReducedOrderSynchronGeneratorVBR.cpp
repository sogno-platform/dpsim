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
}

DP::Ph1::ReducedOrderSynchronGeneratorVBR::ReducedOrderSynchronGeneratorVBR
	(const String & name, Logger::Level logLevel)
	: ReducedOrderSynchronGeneratorVBR(name, name, logLevel) {
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::calculateConductanceMatrix() {
	MatrixFixedSize<2, 2> resistanceMatrix = MatrixFixedSize<2, 2>::Zero(2,2);
	Real DeltaTheta = **mThetaMech - mBase_OmMech * mSimTime;
	resistanceMatrix(0,0) = - (mA + mB) / 2.0 * sin(2*DeltaTheta);
	resistanceMatrix(0,1) = (mA - mB) / 2.0 + (mA + mB) / 2.0 * cos(2*DeltaTheta);
	resistanceMatrix(1,0) = - (mA - mB) / 2.0 + (mA + mB) / 2.0 * cos(2*DeltaTheta);
	resistanceMatrix(1,1) = (mA + mB) / 2.0 * sin(2*DeltaTheta);
	resistanceMatrix = resistanceMatrix * mBase_Z;
	mConductanceMatrix = resistanceMatrix.inverse();
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompInitialize(Real omega,
		Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	Base::ReducedOrderSynchronGenerator<Complex>::mnaCompInitialize(omega, timeStep, leftVector);

	mDomainInterface.setDPShiftFrequency(mBase_OmMech);

	if (mModelAsCurrentSource) {
		// FIXME set variable matrix entries accordingly as shown below
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), matrixNodeIndex(0, 0)));
	} else {
		// get matrix dimension to properly set variable entries
		auto n = leftVector->asRawPointer()->rows();
		auto complexOffset = (UInt)(n / 2);

		// upper left
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex()));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex() + complexOffset, mVirtualNodes[0]->matrixNodeIndex()));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex() + complexOffset));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex() + complexOffset, mVirtualNodes[0]->matrixNodeIndex() + complexOffset));

		// bottom right
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), matrixNodeIndex(0, 0)));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0) + complexOffset, matrixNodeIndex(0, 0)));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), matrixNodeIndex(0, 0) + complexOffset));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0) + complexOffset, matrixNodeIndex(0, 0) + complexOffset));

		// off diagonal
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0)));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex() + complexOffset, matrixNodeIndex(0, 0)));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0) + complexOffset));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(mVirtualNodes[0]->matrixNodeIndex() + complexOffset, matrixNodeIndex(0, 0) + complexOffset));

		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex()));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0) + complexOffset, mVirtualNodes[0]->matrixNodeIndex()));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex() + complexOffset));
		mVariableSystemMatrixEntries.push_back(std::make_pair<UInt,UInt>(matrixNodeIndex(0, 0) + complexOffset, mVirtualNodes[0]->matrixNodeIndex() + complexOffset));
	}

	SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
	for (auto indexPair : mVariableSystemMatrixEntries)
		SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (mModelAsCurrentSource) {
		// Stamp conductance matrix
		// set bottom right block
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrix);

	} else {
		// Stamp voltage source
		Math::setMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[1]->matrixNodeIndex(), Complex(-1, 0));
		Math::setMatrixElement(systemMatrix, mVirtualNodes[1]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));

		// Stamp conductance matrix

		// set upper left block
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), mVirtualNodes[0]->matrixNodeIndex(), mConductanceMatrix);

		// set bottom right block
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductanceMatrix);

		// Set off diagonal blocks
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0, 0), -mConductanceMatrix);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), mVirtualNodes[0]->matrixNodeIndex(), -mConductanceMatrix);
	}
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	if (mModelAsCurrentSource) {
		// compute equivalent northon circuit in abc reference frame
		mIvbr = Complex(mConductanceMatrix(0,0) * mEvbr.real() + mConductanceMatrix(0,1) * mEvbr.imag(),
					    mConductanceMatrix(1,0) * mEvbr.real() + mConductanceMatrix(1,1) * mEvbr.imag());

		Math::setVectorElement(rightVector, matrixNodeIndex(0,0), mIvbr);

	} else {
		Math::setVectorElement(rightVector, mVirtualNodes[1]->matrixNodeIndex(), mEvbr);
	}
}

void DP::Ph1::ReducedOrderSynchronGeneratorVBR::mnaCompPostStep(const Matrix& leftVector) {
	// update armature voltage
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));

	// convert armature voltage to dq reference frame
	**mVdq = mDomainInterface.applyDPToDQTransform((**mIntfVoltage)(0, 0)) / mBase_V_RMS;

	// update armature current
	if (mModelAsCurrentSource) {
		(**mIntfCurrent)(0, 0) = mIvbr - Complex(mConductanceMatrix(0,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrix(0,1) * (**mIntfVoltage)(0, 0).imag(),
					    						 mConductanceMatrix(1,0) * (**mIntfVoltage)(0, 0).real() + mConductanceMatrix(1,1) * (**mIntfVoltage)(0, 0).imag());
	} else {
		(**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[1]->matrixNodeIndex());
	}

	// convert armature current to dq reference frame
	**mIdq = mDomainInterface.applyDPToDQTransform((**mIntfCurrent)(0, 0)) / mBase_I_RMS;
}
