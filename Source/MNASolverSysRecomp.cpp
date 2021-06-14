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

	this->mSLog->info("-- Initialize MNA system matrices and source vector");
	this->mRightSideVector.setZero();

	this->mSLog->info("Number of variable Elements: {}"
		"\nNumber of MNA components: {}",
		this->mVariableComps.size(),
		this->mMNAComponents.size());

	// Save base matrices with only static elements
	this->mSLog->info("Save base matrices (N={}):", this->mSwitches.size());
	for (std::size_t i = 0; i < (1ULL << this->mSwitches.size()); i++){
		auto bit = std::bitset<SWITCH_NUM>(i);
		this->mBaseSystemMatrix[bit][0] = this->mSwitchedMatrices[bit][0];
		this->mSLog->info("{}", Logger::matrixToString(this->mBaseSystemMatrix[bit][0]));
		this->mSLog->flush();
	}

	// Now stamp variable elements into matrix with current switch status
	this->mSLog->info("Stamping variable elements");
	for (auto varElem : this->mMNAIntfVariableComps) {
		varElem->mnaApplySystemMatrixStamp(this->mSwitchedMatrices[this->mCurrentSwitchStatus][0]);
	}

	this->mSLog->info("Initial system matrix with variable elements and current switch status");
	this->mSLog->info("{}", Logger::matrixToString(this->mSwitchedMatrices[this->mCurrentSwitchStatus][0]));
	this->mSLog->flush();

	// Calculate factorization of current matrix
	this->mLuFactorizations[this->mCurrentSwitchStatus][0]->analyzePattern(this->mSwitchedMatrices[this->mCurrentSwitchStatus][0]);
	this->mLuFactorizations[this->mCurrentSwitchStatus][0]->factorize(this->mSwitchedMatrices[this->mCurrentSwitchStatus][0]);
}

template <typename VarType>
void MnaSolverSysRecomp<VarType>::updateVariableCompStatus() {
	for (auto varElem : this->mVariableComps) {
		if (varElem->hasParameterChanged()) {
			auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(varElem);
			this->mSLog->info("Component ({:s} {:s}) value changed -> Update System Matrix",
				idObj->type(), idObj->name());
			mUpdateSysMatrix = true;
			break;
		}
	}
}

template <typename VarType>
void MnaSolverSysRecomp<VarType>::updateSystemMatrix(Real time) {
	this->mSLog->info("Updating System Matrix at {} and switch status {:s}", time, this->mCurrentSwitchStatus.to_string());
	
	// Start from base matrix
	this->mSwitchedMatrices[this->mCurrentSwitchStatus][0] = this->mBaseSystemMatrix[this->mCurrentSwitchStatus][0];

	// Now stamp variable elements into matrix with current switch status
	for (auto comp : this->mMNAIntfVariableComps) {
		comp->mnaApplySystemMatrixStamp(this->mSwitchedMatrices[this->mCurrentSwitchStatus][0]);
		auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
		this->mSLog->debug("Updating {:s} {:s} in system matrix (variabel component)", idObj->type(), idObj->name());
	}

	// Calculate factorization of current matrix
	this->mLuFactorizations[this->mCurrentSwitchStatus][0]->analyzePattern(this->mSwitchedMatrices[this->mCurrentSwitchStatus][0]);
	this->mLuFactorizations[this->mCurrentSwitchStatus][0]->factorize(this->mSwitchedMatrices[this->mCurrentSwitchStatus][0]);
	mUpdateSysMatrix = false;
}

template <typename VarType>
void MnaSolverSysRecomp<VarType>::solve(Real time, Int timeStepCount) {
	// Reset source vector
	this->mRightSideVector.setZero();
	mUpdateSysMatrix = false;

	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

	// Get switch and variable comp status and update system matrix and lu factorization accordingly
	if (!this->mIsInInitialization) {
		this->updateSwitchStatus();
		this->updateVariableCompStatus();
		if (mUpdateSysMatrix)
			this->updateSystemMatrix(time);
	}

	// Calculate new solution vector
	if (this->mSwitchedMatrices.size() > 0)
		this->mLeftSideVector = this->mLuFactorizations[this->mCurrentSwitchStatus][0]->solve(this->mRightSideVector);

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
