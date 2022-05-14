/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/


#include <dpsim/MNASolverEigenPartialKLU.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverEigenPartialKLU<VarType>::MnaSolverEigenPartialKLU(String name, 
CPS::Domain domain, 
CPS::Logger::Level logLevel) :	MnaSolverEigenKLU<VarType>(name, domain, logLevel) {
}
template <typename VarType>
void MnaSolverEigenPartialKLU<VarType>::stampVariableSystemMatrix() {

	this->mSLog->info("Number of variable Elements: {}"
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
	auto start = std::chrono::steady_clock::now();
	this->mLuFactorizationVariableSystemMatrix.factorize_partial(this->mVariableSystemMatrix, this->mListVariableSystemMatrixEntries, 0);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end-start;
	this->mLUTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverEigenPartialKLU<VarType>::solveWithSystemMatrixRecomputation(Real time, Int timeStepCount) {
	//log(time, timeStepCount);
	// Reset source vector
	this->mRightSideVector.setZero();
	
	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

	// Get switch and variable comp status and update system matrix and lu factorization accordingly
	if (this->hasVariableComponentChanged())
		this->recomputeSystemMatrix(time);

	// Calculate new solution vector
	 auto start = std::chrono::steady_clock::now();
	 **this->mLeftSideVector = this->mLuFactorizationVariableSystemMatrix.solve(this->mRightSideVector);

	 auto end = std::chrono::steady_clock::now();
	 std::chrono::duration<double> diff = end-start;
	 this->mSolveTimes.push_back(diff.count());

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
		this->mNodes[nodeIdx]->mnaUpdateVoltage(**this->mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverEigenPartialKLU<VarType>::recomputeSystemMatrix(Real time) {
	// Start from base matrix
	this->mVariableSystemMatrix = this->mBaseSystemMatrix;

	// Now stamp switches into matrix
	for (auto sw : this->mSwitches)
		sw->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

	// Now stamp variable elements into matrix
	for (auto comp : this->mMNAIntfVariableComps)
		comp->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

	// Refactorization of matrix assuming that structure remained
	// constant by omitting analyzePattern
	
	auto start = std::chrono::steady_clock::now();
	// Compute LU-factorization for system matrix
	this->mLuFactorizationVariableSystemMatrix.refactorize_partial(this->mVariableSystemMatrix);
	auto end = std::chrono::steady_clock::now();	
	std::chrono::duration<double> diff = end-start;
	this->mRecomputationTimes.push_back(diff.count());
	++(this->mNumRecomputations);
}
}
template class DPsim::MnaSolverEigenPartialKLU<Real>;
template class DPsim::MnaSolverEigenPartialKLU<Complex>;