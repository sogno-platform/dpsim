// SPDX-License-Identifier: Apache-2.0

#include <dpsim-models/MNASimPowerComp.h>

using namespace CPS;

template<typename VarType>
const Task::List& MNASimPowerComp<VarType>::mnaTasks() const final {
	return mMnaTasks;
}

template<typename VarType>
const Attribute<Matrix>::Ptr MNASimPowerComp<VarType>::getRightVector() const final {
	return mRightVector;
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplySparseSystemMatrixStamp(SparseMatrixRow& systemMatrix) final {
	Matrix mat = Matrix(systemMatrix);
	this->mnaApplySystemMatrixStamp(mat);
	systemMatrix = mat.sparseView();
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaInitialize(Real omega, Real timeStep) final {
	mMnaTasks.clear();
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) final {
	mMnaTasks.clear();
	**this->mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	if (mHasPreStep) {
		this->mMnaTasks.push_back(std::make_shared<typename MNASimPowerComp<VarType>::MnaPreStep>(*this));
	}
	if (mHasPostStep) {
		this->mMnaTasks.push_back(std::make_shared<typename MNASimPowerComp<VarType>::MnaPostStep>(*this, leftVector));
	}

	this->mnaCompInitialize(omega, timeStep, leftVector);
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector) final {
	mMnaTasks.clear();
	this->mnaCompInitializeHarm(omega, timeStep, leftVector);
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplySystemMatrixStamp(Matrix& systemMatrix) final {
	this->mnaCompApplySystemMatrixStamp(systemMatrix);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplyRightSideVectorStamp(Matrix& rightVector) final {
	this->mnaCompApplyRightSideVectorStamp(rightVector);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaUpdateVoltage(const Matrix& leftVector) final {
	this->mnaCompUpdateVoltage(leftVector);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaUpdateCurrent(const Matrix& leftVector) final {
	this->mnaCompUpdateCurrent(leftVector);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaPreStep(Real time, Int timeStepCount) final {
	this->mnaCompPreStep(time, timeStepCount);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) final {
	this->mnaCompPostStep(time, timeStepCount, leftVector);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) final {
	this->mnaCompAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) final {
	this->mnaCompAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) final {
	this->mnaCompApplySystemMatrixStampHarm(systemMatrix, freqIdx);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplyRightSideVectorStampHarm(Matrix& sourceVector) final {
	this->mnaCompApplyRightSideVectorStampHarm(sourceVector);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx) final {
	this->mnaCompApplyRightSideVectorStampHarm(sourceVector, freqIdx);
};

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompUpdateCurrent(const Matrix& leftVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompPreStep(Real time, Int timeStepCount) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompApplyRightSideVectorStampHarm(Matrix& sourceVector) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

template<typename VarType>
void MNASimPowerComp<VarType>::mnaCompApplyRightSideVectorStampHarm(Matrix& sourceVector, Int freqIdx) {
	// Empty default implementation. Can be overridden by child classes if desired.
}

// Declare specializations to move definitions to .cpp
template class CPS::MNASimPowerComp<Real>;
template class CPS::MNASimPowerComp<Complex>;
