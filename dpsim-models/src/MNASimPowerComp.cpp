// SPDX-License-Identifier: Apache-2.0

#include <dpsim-models/MNASimPowerComp.h>

using namespace CPS;

template <typename VarType>
void MNASimPowerComp<VarType>::addMNASubComponent(typename MNASimPowerComp<VarType>::Ptr subc, MNA_SUBCOMP_TASK_ORDER preStepOrder, MNA_SUBCOMP_TASK_ORDER postStepOrder) {
	this->mSubComponents.push_back(subc);
	switch preStepOrder {
		case MNA_SUBCOMP_TASK_ORDER::NO_TASK: break;
		case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
			this->mSubcomponentsBeforePreStep.push_back(subc);
			break;
		}
		case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
			this->mSubcomponentsAfterPreStep.push_back(subc);
			break;
		}
	}
	switch postStepOrder {
		case MNA_SUBCOMP_TASK_ORDER::NO_TASK: break;
		case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
			this->mSubcomponentsBeforePostStep.push_back(subc);
			break;
		}
		case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
			this->mSubcomponentsAfterPostStep.push_back(subc);
			break;
		}
	}
}

// Declare specializations to move definitions to .cpp
template class CPS::MNASimPowerComp<Real>;
template class CPS::MNASimPowerComp<Complex>;
