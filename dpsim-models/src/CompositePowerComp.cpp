// SPDX-License-Identifier: Apache-2.0

#include <dpsim-models/CompositePowerComp.h>

using namespace CPS;

template <typename VarType>
void CompositePowerComp<VarType>::addMNASubComponent(typename SimPowerComp<VarType>::Ptr subc, MNA_SUBCOMP_TASK_ORDER preStepOrder, MNA_SUBCOMP_TASK_ORDER postStepOrder, Bool contributeToRightVector) {
	this->mSubComponents.push_back(subc);
	if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subc)) {
		this->mSubcomponentsMNA.push_back(mnasubcomp);

		if (contributeToRightVector) {
			this->mRightVectorStamps.push_back(mnasubcomp->mRightVector);
		}

		switch (preStepOrder) {
			case MNA_SUBCOMP_TASK_ORDER::NO_TASK: break;
			case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
				this->mSubcomponentsBeforePreStep.push_back(mnasubcomp);
				break;
			}
			case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
				this->mSubcomponentsAfterPreStep.push_back(mnasubcomp);
				break;
			}
		}
		switch (postStepOrder) {
			case MNA_SUBCOMP_TASK_ORDER::NO_TASK: break;
			case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
				this->mSubcomponentsBeforePostStep.push_back(mnasubcomp);
				break;
			}
			case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
				this->mSubcomponentsAfterPostStep.push_back(mnasubcomp);
				break;
			}
		}
	}
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	SimPowerComp<VarType>::updateMatrixNodeIndices();

	for (auto subComp : mSubcomponentsMNA) {
		subComp->mnaInitialize(omega, timeStep, leftVector);
	}

	**this->mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	mnaParentInitialize(omega, timeStep, leftVector);

	if (mHasPreStep) {
		this->mMnaTasks.push_back(std::make_shared<typename MNASimPowerComp<VarType>::MnaPreStep>(*this));
	}
	if (mHasPostStep) {
		this->mMnaTasks.push_back(std::make_shared<typename MNASimPowerComp<VarType>::MnaPostStep>(*this, leftVector));
	}
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (auto subComp : mSubcomponentsMNA) {
		subComp->mnaApplySystemMatrixStamp(systemMatrix);
	}
	mnaParentApplySystemMatrixStamp(systemMatrix);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps) {
		if ((**stamp).size() != 0) {
			rightVector += **stamp;
		}
	}
	mnaParentApplyRightSideVectorStamp(rightVector);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaPreStep(Real time, Int timeStepCount) {
	for (auto subComp : mSubcomponentsBeforePreStep) {
		subComp->mnaPreStep(time, timeStepCount);
	}
	mnaParentPreStep(time, timeStepCount);
	for (auto subComp : mSubcomponentsAfterPreStep) {
		subComp->mnaPreStep(time, timeStepCount);
	}
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	for (auto subComp : mSubcomponentsBeforePostStep) {
		subComp->mnaPostStep(time, timeStepCount, leftVector);
	}
	mnaParentPostStep(time, timeStepCount, leftVector);
	for (auto subComp : mSubcomponentsAfterPostStep) {
		subComp->mnaPostStep(time, timeStepCount, leftVector);
	}
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	for (auto subComp : mSubcomponentsMNA) {
		subComp->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	}
	mnaParentAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	for (auto subComp : mSubcomponentsMNA) {
		subComp->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	}
	mnaParentAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
}

// Declare specializations to move definitions to .cpp
template class CPS::CompositePowerComp<Real>;
template class CPS::CompositePowerComp<Complex>;
