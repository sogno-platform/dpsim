/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverDirect.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {


template <typename VarType>
MnaSolverDirect<VarType>::MnaSolverDirect(String name, CPS::Domain domain, CPS::Logger::Level logLevel) :	MnaSolver<VarType>(name, domain, logLevel) {
}


template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixEmpty(std::size_t index) {
	mSwitchedMatrices[std::bitset<SWITCH_NUM>(index)][0].setZero();
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixEmpty(std::size_t swIdx, Int freqIdx) {
	mSwitchedMatrices[std::bitset<SWITCH_NUM>(swIdx)][freqIdx].setZero();
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixStamp(std::size_t index, std::vector<std::shared_ptr<CPS::MNAInterface>>& comp)
{
	auto bit = std::bitset<SWITCH_NUM>(index);
	auto& sys = mSwitchedMatrices[bit][0];
	for (auto comp : comp) {
		comp->mnaApplySparseSystemMatrixStamp(sys);
	}
	for (UInt i = 0; i < mSwitches.size(); ++i)
		mSwitches[i]->mnaApplySwitchSparseSystemMatrixStamp(bit[i], sys, 0);

	// Compute LU-factorization for system matrix
	mLuFactorizations[bit][0]->preprocessing(sys, mListVariableSystemMatrixEntries);

	auto start = std::chrono::steady_clock::now();
	mLuFactorizations[bit][0]->factorize(sys);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end-start;
	mLUTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverDirect<VarType>::stampVariableSystemMatrix() {

	this->mLuFactorizationVariableSystemMatrix = createDirectSolverImplementation();

	mSLog->info("Number of variable Elements: {}"
				"\nNumber of MNA components: {}",
				mVariableComps.size(),
				mMNAComponents.size());

	// Build base matrix with only static elements
	mBaseSystemMatrix.setZero();
	for (auto statElem : mMNAComponents)
		statElem->mnaApplySparseSystemMatrixStamp(mBaseSystemMatrix);
	mSLog->info("Base matrix with only static elements: {}", Logger::matrixToString(mBaseSystemMatrix));
	mSLog->flush();

	// Use matrix with only static elements as basis for variable system matrix
	mVariableSystemMatrix = mBaseSystemMatrix;

	// Now stamp switches into matrix
	mSLog->info("Stamping switches");
	for (auto sw : mMNAIntfSwitches)
		sw->mnaApplySparseSystemMatrixStamp(mVariableSystemMatrix);

	// Now stamp initial state of variable elements into matrix
	mSLog->info("Stamping variable elements");
	for (auto varElem : mMNAIntfVariableComps)
		varElem->mnaApplySparseSystemMatrixStamp(mVariableSystemMatrix);

	mSLog->info("Initial system matrix with variable elements {}", Logger::matrixToString(mVariableSystemMatrix));
	mSLog->flush();

	// Calculate factorization of current matrix
	mLuFactorizationVariableSystemMatrix->preprocessing(mVariableSystemMatrix, mListVariableSystemMatrixEntries);
	
	auto start = std::chrono::steady_clock::now();
	mLuFactorizationVariableSystemMatrix->factorize(mVariableSystemMatrix);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end-start;
	mLUTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverDirect<VarType>::solveWithSystemMatrixRecomputation(Real time, Int timeStepCount) {
	// Reset source vector
	mRightSideVector.setZero();

	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVector += *stamp;

	// Get switch and variable comp status and update system matrix and lu factorization accordingly
	if (hasVariableComponentChanged())
		recomputeSystemMatrix(time);

	auto start = std::chrono::steady_clock::now();
	// Calculate new solution vector
	**mLeftSideVector = mLuFactorizationVariableSystemMatrix->solve(mRightSideVector);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end-start;
	mSolveTimes.push_back(diff.count());

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
		mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverDirect<VarType>::recomputeSystemMatrix(Real time) {
	// Start from base matrix
	mVariableSystemMatrix = mBaseSystemMatrix;

	// Now stamp switches into matrix
	for (auto sw : mMNAIntfSwitches)
		sw->mnaApplySparseSystemMatrixStamp(mVariableSystemMatrix);

	// Now stamp variable elements into matrix
	for (auto comp : mMNAIntfVariableComps)
		comp->mnaApplySparseSystemMatrixStamp(mVariableSystemMatrix);

	auto start = std::chrono::steady_clock::now();
	// Refactorization of matrix assuming that structure remained
	// constant by omitting analyzePattern
	mLuFactorizationVariableSystemMatrix->partialRefactorize(mVariableSystemMatrix, mListVariableSystemMatrixEntries);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<double> diff = end-start;
	mRecomputationTimes.push_back(diff.count());
	++mNumRecomputations;
}

template<>
void MnaSolverDirect<Real>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	if (mSystemMatrixRecomputation) {
		mBaseSystemMatrix = SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
		mVariableSystemMatrix = SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
	} else {
		for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++){
			auto bit = std::bitset<SWITCH_NUM>(i);
			mSwitchedMatrices[bit].push_back(SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices));
			mLuFactorizations[bit].push_back(createDirectSolverImplementation());
		}
	}
}

template<>
void MnaSolverDirect<Complex>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	if (mFrequencyParallel) {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); ++i) {
			for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
				auto bit = std::bitset<SWITCH_NUM>(i);
				mSwitchedMatrices[bit].push_back(SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices)));
				mLuFactorizations[bit].push_back(createDirectSolverImplementation());
			}
		}
	} else if (mSystemMatrixRecomputation) {
		mBaseSystemMatrix = SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices));
		mVariableSystemMatrix = SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices));
	} else {
		for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
			auto bit = std::bitset<SWITCH_NUM>(i);
			mSwitchedMatrices[bit].push_back(SparseMatrix(2*(mNumTotalMatrixNodeIndices), 2*(mNumTotalMatrixNodeIndices)));
			mLuFactorizations[bit].push_back(createDirectSolverImplementation());
		}
	}
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createSolveTask()
{
	return std::make_shared<MnaSolverDirect<VarType>::SolveTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createSolveTaskRecomp()
{
	return std::make_shared<MnaSolverDirect<VarType>::SolveTaskRecomp>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createSolveTaskHarm(UInt freqIdx)
{
	return std::make_shared<MnaSolverDirect<VarType>::SolveTaskHarm>(*this, freqIdx);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createLogTask()
{
	return std::make_shared<MnaSolverDirect<VarType>::LogTask>(*this);
}

template <typename VarType>
void MnaSolverDirect<VarType>::solve(Real time, Int timeStepCount) {
	// Reset source vector
	mRightSideVector.setZero();

	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVector += *stamp;

	if (!mIsInInitialization)
		MnaSolver<VarType>::updateSwitchStatus();

	if (mSwitchedMatrices.size() > 0) {
		auto start = std::chrono::steady_clock::now();
		**mLeftSideVector = mLuFactorizations[mCurrentSwitchStatus][0]->solve(mRightSideVector);
		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<double> diff = end-start;
		mSolveTimes.push_back(diff.count());
	}


	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
		mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverDirect<VarType>::solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx) {
	mRightSideVectorHarm[freqIdx].setZero();

	// Sum of right side vectors (computed by the components' pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVectorHarm[freqIdx] += stamp->col(freqIdx);

	**mLeftSideVectorHarm[freqIdx] = mLuFactorizations[mCurrentSwitchStatus][freqIdx]->solve(mRightSideVectorHarm[freqIdx]);
}

template <typename VarType>
void MnaSolverDirect<VarType>::logSystemMatrices() {
	if (mFrequencyParallel) {
		for (UInt i = 0; i < mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)].size(); ++i) {
			mSLog->info("System matrix for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][i]));
			//mSLog->info("LU decomposition for frequency: {:d} \n{:s}", i,
			//	Logger::matrixToString(mLuFactorizations[std::bitset<SWITCH_NUM>(0)][i]->matrixLU()));
		}

		for (UInt i = 0; i < mRightSideVectorHarm.size(); ++i)
			mSLog->info("Right side vector for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mRightSideVectorHarm[i]));

	}
	else if (mSystemMatrixRecomputation) {
		mSLog->info("Summarizing matrices: ");
		mSLog->info("Base matrix with only static elements: {}", Logger::matrixToString(mBaseSystemMatrix));
		mSLog->info("Initial system matrix with variable elements {}", Logger::matrixToString(mVariableSystemMatrix));
		mSLog->info("Right side vector: {}", Logger::matrixToString(mRightSideVector));
	} else {
		if (mSwitches.size() < 1) {
			mSLog->info("System matrix: \n{}", mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][0]);
			//mSLog->info("LU decomposition: \n{}",	mLuFactorizations[std::bitset<SWITCH_NUM>(0)]->matrixLU());
		}
		else {
			mSLog->info("Initial switch status: {:s}", mCurrentSwitchStatus.to_string());

			for (auto sys : mSwitchedMatrices) {
				mSLog->info("Switching System matrix {:s} \n{:s}",
					sys.first.to_string(), Logger::matrixToString(sys.second[0]));
				//mSLog->info("LU Factorization for System Matrix {:s} \n{:s}",
				//	sys.first.to_string(), Logger::matrixToString(mLuFactorizations[sys.first]->matrixLU()));
			}
		}
		mSLog->info("Right side vector: \n{}", mRightSideVector);
	}
}

template<typename VarType>
void MnaSolverDirect<VarType>::logSolveTime(){
	Real solveSum = 0.0;
	Real solveMax = 0.0;
	for(auto meas : mSolveTimes)
	{
		solveSum += meas;
		if(meas > solveMax)
			solveMax = meas;
	}
    mSLog->info("Cumulative solve times: {:.12f}",solveSum);
	mSLog->info("Average solve time: {:.12f}", solveSum/mSolveTimes.size());
	mSLog->info("Maximum solve time: {:.12f}", solveMax);
	mSLog->info("Number of solves: {:d}", mSolveTimes.size());
}


template <typename VarType>
void MnaSolverDirect<VarType>::logLUTime()
{
	for (auto meas : mLUTimes) {
		mSLog->info("LU factorization time: {:.12f}", meas);
	}
}

template <typename VarType>
void MnaSolverDirect<VarType>::logRecomputationTime(){
       Real recompSum = 0.0;
	   Real recompMax = 0.0;
       for (auto meas : mRecomputationTimes){
               recompSum += meas;
			   if(meas > recompMax)
					recompMax = meas;
       }
	   // Sometimes, refactorization is not used
	   if(mRecomputationTimes.size() != 0){
		    mSLog->info("Cumulative refactorization times: {:.12f}",recompSum);
       		mSLog->info("Average refactorization time: {:.12f}", recompSum/mRecomputationTimes.size());
			mSLog->info("Maximum refactorization time: {:.12f}", recompMax);
       		mSLog->info("Number of refactorizations: {:d}", mRecomputationTimes.size());
	   }
}

template<typename VarType>
std::shared_ptr<DirectLinearSolver> MnaSolverDirect<VarType>::createDirectSolverImplementation() {
	switch(this->implementationInUse)
	{
		case DirectLinearSolverImpl::DenseLU:
			return std::make_shared<DenseLUAdapter>();
		#ifdef WITH_SPARSE
		case DirectLinearSolverImpl::SparseLU:
			return std::make_shared<SparseLUAdapter>();
		#endif
		#ifdef WITH_KLU
		case DirectLinearSolverImpl::KLU:
			return std::make_shared<KLUAdapter>();
		#endif
		#ifdef WITH_CUDA
		case DirectLinearSolverImpl::CUDADense:
			return std::make_shared<GpuDenseAdapter>();
		#ifdef WITH_SPARSE
		case DirectLinearSolverImpl::CUDASparse:
			return std::make_shared<GpuSparseAdapter>();
		#endif
		#ifdef WITH_MAGMA
		case DirectLinearSolverImpl::CUDAMagma:
			return std::make_shared<GpuMagmaAdapter>();
		#endif
		#endif
		default:
			throw CPS::SystemError("unsupported linear solver implementation.");
	}
}

template <typename VarType>
void MnaSolverDirect<VarType>::setDirectLinearSolverImplementation(DirectLinearSolverImpl implementation) {
	this->implementationInUse = implementation;
	//this->mLuFactorizationVariableSystemMatrix = createDirectSolverImplementation();
}

}

template class DPsim::MnaSolverDirect<Real>;
template class DPsim::MnaSolverDirect<Complex>;
