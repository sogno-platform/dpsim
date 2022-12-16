// SPDX-License-Identifier: Apache-2.0

#include <dpsim-models/MNASimPowerComp.h>

using namespace CPS;

template<typename VarType>
const Task::List& MNASimPowerComp<VarType>::mnaTasks() {
	return mMnaTasks;
}

const Attribute<Matrix>::Ptr getRightVector() {
	return mRightVector;
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplyRightSideVectorStamp(Matrix& rightVector) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaUpdateVoltage(const Matrix& leftVector) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaUpdateCurrent(const Matrix& leftVector) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaPreStep(Real time, Int timeStepCount) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplyRightSideVectorStampHarm(Matrix& sourceVector) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx) {

};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplySparseSystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	Matrix mat = Matrix(systemMatrix);
	mnaApplySystemMatrixStamp(mat);
	systemMatrix = mat.sparseView();
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaInitialize(Real omega, Real timeStep) {
	mMnaTasks.clear();
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mMnaTasks.clear();
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector) {
	mMnaTasks.clear();
}

// Declare specializations to move definitions to .cpp
template class CPS::MNASimPowerComp<Real>;
template class CPS::MNASimPowerComp<Complex>;
