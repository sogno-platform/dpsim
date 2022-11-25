// SPDX-License-Identifier: Apache-2.0

#include <dpsim-models/MNASimPowerComp.h>

using namespace CPS;

template <typename VarType>
void MNASimPowerComp<VarType>::addMNASubComponent(typename SimPowerComp<VarType>::Ptr subc, MNA_SUBCOMP_TASK_ORDER preStepOrder, MNA_SUBCOMP_TASK_ORDER postStepOrder) {
	this->mSubComponents.push_back(subc);
	if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subc)) {
		this->mSubcomponentsMNA.push_back(mnasubcomp);
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

// template <typename VarType>
// void MNASimPowerComp<VarType>::addMNASubComponent(typename MNASimPowerComp<VarType>::Ptr subc, MNA_SUBCOMP_TASK_ORDER preStepOrder, MNA_SUBCOMP_TASK_ORDER postStepOrder) {
// 	this->mSubComponents.push_back(subc);
// 	this->mSubcomponentsMNA.push_back(subc);
// 	switch (preStepOrder) {
// 		case MNA_SUBCOMP_TASK_ORDER::NO_TASK: break;
// 		case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
// 			this->mSubcomponentsBeforePreStep.push_back(subc);
// 			break;
// 		}
// 		case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
// 			this->mSubcomponentsAfterPreStep.push_back(subc);
// 			break;
// 		}
// 	}
// 	switch (postStepOrder) {
// 		case MNA_SUBCOMP_TASK_ORDER::NO_TASK: break;
// 		case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
// 			this->mSubcomponentsBeforePostStep.push_back(subc);
// 			break;
// 		}
// 		case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
// 			this->mSubcomponentsAfterPostStep.push_back(subc);
// 			break;
// 		}
// 	}
// }

template <typename VarType>
void MNASimPowerComp<VarType>::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	if (this->hasSubComponents()) {
		for (auto subComp : mSubcomponentsMNA) {
			subComp->mnaInitialize(omega, timeStep, leftVector);
		}
		mnaParentInitialize(omega, timeStep, leftVector);
	}
}

template <typename VarType>
void MNASimPowerComp<VarType>::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (this->hasSubComponents()) {
		for (auto subComp : mSubcomponentsMNA) {
			subComp->mnaApplySystemMatrixStamp(systemMatrix);
		}
		mnaParentApplySystemMatrixStamp(systemMatrix);
	}
}

template <typename VarType>
void MNASimPowerComp<VarType>::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (this->hasSubComponents()) {
		for (auto subComp : mSubcomponentsMNA) {
			subComp->mnaApplyRightSideVectorStamp(rightVector);
		}
		mnaParentApplyRightSideVectorStamp(rightVector);
	}
}

template <typename VarType>
void MNASimPowerComp<VarType>::mnaPreStep(Real time, Int timeStepCount) {
	if (this->hasSubComponents()) {
		for (auto subComp : mSubcomponentsBeforePreStep) {
			subComp->mnaPreStep(time, timeStepCount);
		}
		mnaParentPreStep(time, timeStepCount);
		for (auto subComp : mSubcomponentsAfterPreStep) {
			subComp->mnaPreStep(time, timeStepCount);
		}
	}
}

template <typename VarType>
void MNASimPowerComp<VarType>::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	if (this->hasSubComponents()) {
		for (auto subComp : mSubcomponentsBeforePostStep) {
			subComp->mnaPostStep(time, timeStepCount, leftVector);
		}
		mnaParentPostStep(time, timeStepCount, leftVector);
		for (auto subComp : mSubcomponentsAfterPostStep) {
			subComp->mnaPostStep(time, timeStepCount, leftVector);
		}
	}
}

template <typename VarType>
void MNASimPowerComp<VarType>::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	if (this->hasSubComponents()) {
		for (auto subComp : mSubcomponentsMNA) {
			subComp->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
		}
		mnaParentAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	}
}

template <typename VarType>
void MNASimPowerComp<VarType>::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	if (this->hasSubComponents()) {
		for (auto subComp : mSubcomponentsMNA) {
			subComp->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
		}
		mnaParentAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	}
}

// Declare specializations to move definitions to .cpp
template class CPS::MNASimPowerComp<Real>;
template class CPS::MNASimPowerComp<Complex>;