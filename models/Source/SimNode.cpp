/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SimNode.h>

using namespace CPS;

template <typename VarType>
SimNode<VarType>::SimNode(String uid, String name,
	std::vector<UInt> matrixNodeIndex, PhaseType phaseType, const std::vector<Complex> &initialVoltage)
	: TopologicalNode(uid, name, phaseType, initialVoltage),
	mVoltage(Attribute<MatrixVar<VarType>>::create("v", mAttributes)),
	mApparentPower(Attribute<MatrixVar<VarType>>::create("s", mAttributes)) {

	if (phaseType == PhaseType::ABC) {
		mMatrixNodeIndex = matrixNodeIndex;
		**mVoltage = MatrixVar<VarType>::Zero(3, 1);
		**mApparentPower = MatrixVar<VarType>::Zero(3, 1);
	}
	else {
		mMatrixNodeIndex[0] = matrixNodeIndex[0];
		**mVoltage = MatrixVar<VarType>::Zero(1, 1);
		**mApparentPower = MatrixVar<VarType>::Zero(1, 1);
	}
}

template <typename VarType>
SimNode<VarType>::SimNode(PhaseType phaseType)
	: SimNode("gnd", "gnd", { 0, 0, 0 }, phaseType, { 0, 0, 0 }) {
		mIsGround = true;
		**mInitialVoltage = MatrixComp::Zero(3, 1);
		**mVoltage = MatrixVar<VarType>::Zero(3, 1);
}

template <typename VarType>
void SimNode<VarType>::initialize(Matrix frequencies) {
	mFrequencies = frequencies;
	mNumFreqs = static_cast<UInt>(mFrequencies.size());
	Matrix::Index rowNum = phaseType() == PhaseType::ABC ? 3 : 1;
	**mVoltage = MatrixVar<VarType>::Zero(rowNum, mNumFreqs);
}

template <typename VarType>
VarType SimNode<VarType>::singleVoltage(PhaseType phaseType) {
	if (phaseType == PhaseType::B)
		return (**mVoltage)(1,0);
	else if (phaseType == PhaseType::C)
		return (**mVoltage)(2,0);
	else // phaseType == PhaseType::Single || mPhaseType == PhaseType::A
		return (**mVoltage)(0,0);
}

template<>
void SimNode<Complex>::setVoltage(Complex newVoltage) {
	(**mVoltage)(0, 0) = newVoltage;
}

template<>
void SimNode<Complex>::setPower(Complex newPower) {
	(**mApparentPower)(0, 0) = newPower;
}

template<>
void SimNode<Real>::mnaUpdateVoltage(const Matrix& leftVector) {
	if (mMatrixNodeIndex[0] >= 0) (**mVoltage)(0,0) = Math::realFromVectorElement(leftVector, mMatrixNodeIndex[0]);
	if (mPhaseType == PhaseType::ABC) {
		if (mMatrixNodeIndex[1] >= 0) (**mVoltage)(1,0) = Math::realFromVectorElement(leftVector, mMatrixNodeIndex[1]);
		if (mMatrixNodeIndex[2] >= 0) (**mVoltage)(2,0) = Math::realFromVectorElement(leftVector, mMatrixNodeIndex[2]);
	}
}

template<>
void SimNode<Complex>::mnaUpdateVoltage(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
			if (mMatrixNodeIndex[0] >= 0) (**mVoltage)(0,freq) = Math::complexFromVectorElement(leftVector, mMatrixNodeIndex[0], mNumFreqs, freq);
		if (mPhaseType == PhaseType::ABC) {
			if (mMatrixNodeIndex[1] >= 0) (**mVoltage)(1,freq) = Math::complexFromVectorElement(leftVector, mMatrixNodeIndex[1], mNumFreqs, freq);
			if (mMatrixNodeIndex[2] >= 0) (**mVoltage)(2,freq) = Math::complexFromVectorElement(leftVector, mMatrixNodeIndex[2], mNumFreqs, freq);
		}
	}
}

template<>
void SimNode<Real>::mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) { }

template<>
void SimNode<Complex>::mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) {
	if (mMatrixNodeIndex[0] >= 0) (**mVoltage)(0,freqIdx) = Math::complexFromVectorElement(leftVector, mMatrixNodeIndex[0]);
	if (mPhaseType == PhaseType::ABC) {
		if (mMatrixNodeIndex[1] >= 0) (**mVoltage)(1,freqIdx) = Math::complexFromVectorElement(leftVector, mMatrixNodeIndex[1]);
		if (mMatrixNodeIndex[2] >= 0) (**mVoltage)(2,freqIdx) = Math::complexFromVectorElement(leftVector, mMatrixNodeIndex[2]);
	}
}

template <>
void SimNode<Real>::mnaInitializeHarm(std::vector<Attribute<Matrix>::Ptr> leftVectors) { }

template <>
void SimNode<Complex>::mnaInitializeHarm(std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	mMnaTasks = {
		std::make_shared<MnaPostStepHarm>(*this, leftVectors)
	};
}

template <>
void SimNode<Complex>::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
	for (UInt freq = 0; freq < mNode.mNumFreqs; freq++)
		mNode.mnaUpdateVoltageHarm(**mLeftVectors[freq], freq);
}

template <>
void SimNode<Real>::MnaPostStepHarm::execute(Real time, Int timeStepCount) {

}

template class CPS::SimNode<Real>;
template class CPS::SimNode<Complex>;
