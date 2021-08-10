/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverSysRecomp.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverSysRecomp<VarType>::MnaSolverSysRecomp(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolverEigenSparse<VarType>(name, domain, logLevel) { }

template <typename VarType>
void MnaSolverSysRecomp<VarType>::initializeSystem() {
	MnaSolver<VarType>::initializeSystem();

	this->mSLog->info(	"Number of variable Elements: {}"
						"\nNumber of MNA components: {}",
						this->mVariableComps.size(),
						this->mMNAComponents.size());

	// Build base matrix with only static elements
	this->mBaseSystemMatrix.setZero();
	for (auto statElem : this->mMNAComponents)
		statElem->mnaApplySystemMatrixStamp(this->mBaseSystemMatrix);
	this->mSLog->info("Base matrix with only static elements: {}", Logger::matrixToString(this->mBaseSystemMatrix));
	this->mSLog->flush();
	
	// Use matrix with only static elements as basis for variable system matrix
	this->mVariableSystemMatrix = this->mBaseSystemMatrix;

	// Now stamp switches into matrix
	this->mSLog->info("Stamping switches");
	for (auto sw : this->mSwitches)
		sw->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

	// Now stamp initial state of variable elements into matrix
	this->mSLog->info("Stamping variable elements");
	for (auto varElem : this->mMNAIntfVariableComps)
		varElem->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

	this->mSLog->info("Initial system matrix with variable elements {}", Logger::matrixToString(this->mVariableSystemMatrix));
	this->mSLog->flush();

	// Calculate factorization of current matrix
	this->mLuFactorizationVariableSystemMatrix.analyzePattern(this->mVariableSystemMatrix);
	this->mLuFactorizationVariableSystemMatrix.factorize(this->mVariableSystemMatrix);
}

template <typename VarType>
Bool MnaSolverSysRecomp<VarType>::hasVariableComponentChanged() {
	for (auto varElem : this->mVariableComps) {
		if (varElem->hasParameterChanged()) {
			auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(varElem);
			this->mSLog->info("Component ({:s} {:s}) value changed -> Update System Matrix",
				idObj->type(), idObj->name());
			return true;
		}
	}
	return false;
}

template <typename VarType>
void MnaSolverSysRecomp<VarType>::updateSystemMatrix(Real time) {
	// Start from base matrix
	this->mVariableSystemMatrix = this->mBaseSystemMatrix;

	// Now stamp switches into matrix
	for (auto sw : this->mSwitches)
		sw->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

	// Now stamp variable elements into matrix
	for (auto comp : this->mMNAIntfVariableComps)
		comp->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

	// Calculate factorization of current matrix
	this->mLuFactorizationVariableSystemMatrix.analyzePattern(this->mVariableSystemMatrix);
	this->mLuFactorizationVariableSystemMatrix.factorize(this->mVariableSystemMatrix);
}

template <typename VarType>
void MnaSolverSysRecomp<VarType>::solve(Real time, Int timeStepCount) {
	// Reset source vector
	this->mRightSideVector.setZero();
	
	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

	// Get switch and variable comp status and update system matrix and lu factorization accordingly
	if (this->hasVariableComponentChanged())
		this->updateSystemMatrix(time);

	// Calculate new solution vector
	this->mLeftSideVector = this->mLuFactorizationVariableSystemMatrix.solve(this->mRightSideVector);

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
		this->mNodes[nodeIdx]->mnaUpdateVoltage(this->mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
Task::List MnaSolverSysRecomp<VarType>::getTasks() {
	Task::List l;

	for (auto comp : this->mMNAComponents) {
		for (auto task : comp->mnaTasks())
			l.push_back(task);
	}
	for (auto comp : this->mMNAIntfVariableComps) {
		for (auto task : comp->mnaTasks())
			l.push_back(task);
	}
	for (auto node : this->mNodes) {
		for (auto task : node->mnaTasks())
			l.push_back(task);
	}
	// TODO signal components should be moved out of MNA solver
	for (auto comp : this->mSimSignalComps) {
		for (auto task : comp->getTasks())
			l.push_back(task);
	}

	l.push_back(std::make_shared<MnaSolverSysRecomp<VarType>::SolveTask>(*this));
	l.push_back(std::make_shared<MnaSolverSysRecomp<VarType>::LogTask>(*this));
	return l;
}

}

template class DPsim::MnaSolverSysRecomp<Real>;
template class DPsim::MnaSolverSysRecomp<Complex>;
