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
	mImplementationInUse = DirectLinearSolverImpl::SparseLU;
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
	for (auto component : comp) {
		component->mnaApplySystemMatrixStamp(sys);
	}
	for (UInt i = 0; i < mSwitches.size(); ++i)
		mSwitches[i]->mnaApplySwitchSystemMatrixStamp(bit[i], sys, 0);

	// Compute LU-factorization for system matrix
	mDirectLinearSolvers[bit][0]->preprocessing(sys, mListVariableSystemMatrixEntries);
	auto start = std::chrono::steady_clock::now();
	mDirectLinearSolvers[bit][0]->factorize(sys);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<Real> diff = end-start;
	mFactorizeTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverDirect<VarType>::stampVariableSystemMatrix() {

	this->mDirectLinearSolverVariableSystemMatrix = createDirectSolverImplementation(mSLog);
	// TODO: a direct linear solver configuration is only applied if system matrix recomputation is used
	this->mDirectLinearSolverVariableSystemMatrix->setConfiguration(mConfigurationInUse);

	SPDLOG_LOGGER_INFO(mSLog, "Number of variable Elements: {}"
				"\nNumber of MNA components: {}",
				mVariableComps.size(),
				mMNAComponents.size());

	// Build base matrix with only static elements
	mBaseSystemMatrix.setZero();
	for (auto statElem : mMNAComponents)
		statElem->mnaApplySystemMatrixStamp(mBaseSystemMatrix);
	SPDLOG_LOGGER_INFO(mSLog, "Base matrix with only static elements: {}", Logger::matrixToString(mBaseSystemMatrix));
	mSLog->flush();

	// Continue from base matrix
	mVariableSystemMatrix = mBaseSystemMatrix;

	// Now stamp switches into matrix
	SPDLOG_LOGGER_INFO(mSLog, "Stamping switches");
	for (auto sw : mMNAIntfSwitches)
		sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	// Now stamp initial state of variable elements into matrix
	SPDLOG_LOGGER_INFO(mSLog, "Stamping variable elements");
	for (auto varElem : mMNAIntfVariableComps)
		varElem->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	SPDLOG_LOGGER_INFO(mSLog, "Initial system matrix with variable elements {}", Logger::matrixToString(mVariableSystemMatrix));
	/* TODO: find replacement for flush() */
	mSLog->flush();

	// Calculate factorization of current matrix
	mDirectLinearSolverVariableSystemMatrix->preprocessing(mVariableSystemMatrix, mListVariableSystemMatrixEntries);

	auto start = std::chrono::steady_clock::now();
	mDirectLinearSolverVariableSystemMatrix->factorize(mVariableSystemMatrix);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<Real> diff = end-start;
	mFactorizeTimes.push_back(diff.count());
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

	// Calculate new solution vector
	auto start = std::chrono::steady_clock::now();
	**mLeftSideVector = mDirectLinearSolverVariableSystemMatrix->solve(mRightSideVector);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<Real> diff = end-start;
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
		sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	// Now stamp variable elements into matrix
	for (auto comp : mMNAIntfVariableComps)
		comp->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	// Refactorization of matrix assuming that structure remained
	// constant by omitting analyzePattern
	auto start = std::chrono::steady_clock::now();
	mDirectLinearSolverVariableSystemMatrix->partialRefactorize(mVariableSystemMatrix, mListVariableSystemMatrixEntries);
	auto end = std::chrono::steady_clock::now();
	std::chrono::duration<Real> diff = end-start;
	mRecomputationTimes.push_back(diff.count());
	++mNumRecomputations;
}

template<>
void MnaSolverDirect<Real>::createEmptySystemMatrix() {
	auto mSolverParamsMNA = getMNAParameters(); 
	if (mSolverParamsMNA != nullptr) {
		if (mSwitches.size() > SWITCH_NUM)
			throw SystemError("Too many Switches.");

		if (mSolverParamsMNA->mSystemMatrixRecomputation) {
			mBaseSystemMatrix = SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
			mVariableSystemMatrix = SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
		} else {
			for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++){
				auto bit = std::bitset<SWITCH_NUM>(i);
				mSwitchedMatrices[bit].push_back(SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices));
				mDirectLinearSolvers[bit].push_back(createDirectSolverImplementation(mSLog));
			}
		}
	}
}

template<>
void MnaSolverDirect<Complex>::createEmptySystemMatrix() {
	auto mSolverParamsMNA = getMNAParameters(); 
	if (mSolverParamsMNA != nullptr) {
		if (mSwitches.size() > SWITCH_NUM)
			throw SystemError("Too many Switches.");

		if (mSolverParamsMNA->mFreqParallel) {
			for (UInt i = 0; i < std::pow(2,mSwitches.size()); ++i) {
				for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
					auto bit = std::bitset<SWITCH_NUM>(i);
					mSwitchedMatrices[bit].push_back(SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices)));
					mDirectLinearSolvers[bit].push_back(createDirectSolverImplementation(mSLog));
				}
			}
		} else if (mSolverParamsMNA->mSystemMatrixRecomputation) {
			mBaseSystemMatrix = SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices));
			mVariableSystemMatrix = SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices));
		} else {
			for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
				auto bit = std::bitset<SWITCH_NUM>(i);
				mSwitchedMatrices[bit].push_back(SparseMatrix(2*(mNumTotalMatrixNodeIndices), 2*(mNumTotalMatrixNodeIndices)));
				mDirectLinearSolvers[bit].push_back(createDirectSolverImplementation(mSLog));
			}
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

	// Add together the right side vector (computed by the components' pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVector += *stamp;

	if (!mIsInInitialization)
		MnaSolver<VarType>::updateSwitchStatus();

	if (mSwitchedMatrices.size() > 0){
		auto start = std::chrono::steady_clock::now();
		**mLeftSideVector = mDirectLinearSolvers[mCurrentSwitchStatus][0]->solve(mRightSideVector);
		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<Real> diff = end-start;
		mSolveTimes.push_back(diff.count());
	}

	// CHECK: Is this really required? Or can operations actually become part of
	// correctorStep and mnaPostStep?
	for (auto syncGen : mSyncGen)
		syncGen->updateVoltage(**mLeftSideVector);

	// Reset number of iterations
	mIter = 0;

	// Additional solve steps for iterative models
	if (mSyncGen.size() > 0) {
		UInt numCompsRequireIter;
		do {
			// count synchronous generators that require iteration
			numCompsRequireIter = 0;
			for (auto syncGen : mSyncGen)
				if (syncGen->requiresIteration())
					numCompsRequireIter++;

			// recompute solve step if at least one component demands iteration
			if (numCompsRequireIter > 0){
				mIter++;

				// Reset source vector
				mRightSideVector.setZero();

				if (!mIsInInitialization)
					MnaSolver<VarType>::updateSwitchStatus();

				for (auto syncGen : mSyncGen)
					syncGen->correctorStep();

				// Add together the right side vector (computed by the components' pre-step tasks)
				for (auto stamp : mRightVectorStamps)
					mRightSideVector += *stamp;

				if (mSwitchedMatrices.size() > 0) {
					auto start = std::chrono::steady_clock::now();
					**mLeftSideVector = mDirectLinearSolvers[mCurrentSwitchStatus][0]->solve(mRightSideVector);
					auto end = std::chrono::steady_clock::now();
					std::chrono::duration<Real> diff = end-start;
					mSolveTimes.push_back(diff.count());
				}

				// CHECK: Is this really required? Or can operations actually become part of
				// correctorStep and mnaPostStep?
				for (auto syncGen : mSyncGen)
					syncGen->updateVoltage(**mLeftSideVector);
			}
		}
		while (numCompsRequireIter > 0);
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

	**mLeftSideVectorHarm[freqIdx] = mDirectLinearSolvers[mCurrentSwitchStatus][freqIdx]->solve(mRightSideVectorHarm[freqIdx]);
}

template <typename VarType>
void MnaSolverDirect<VarType>::logSystemMatrices() {
	auto mSolverParamsMNA = getMNAParameters(); 
	if (mSolverParamsMNA != nullptr) {
		if (mSolverParamsMNA->mFreqParallel) {
			for (UInt i = 0; i < mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)].size(); ++i) {
				SPDLOG_LOGGER_INFO(mSLog, "System matrix for frequency: {:d} \n{:s}", i, Logger::matrixToString(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][i]));
			}

			for (UInt i = 0; i < mRightSideVectorHarm.size(); ++i) {
				SPDLOG_LOGGER_INFO(mSLog, "Right side vector for frequency: {:d} \n{:s}", i, Logger::matrixToString(mRightSideVectorHarm[i]));
			}

		}
		else if (mSolverParamsMNA->mSystemMatrixRecomputation) {
			SPDLOG_LOGGER_INFO(mSLog, "Summarizing matrices: ");
			SPDLOG_LOGGER_INFO(mSLog, "Base matrix with only static elements: {}", Logger::matrixToString(mBaseSystemMatrix));
			SPDLOG_LOGGER_INFO(mSLog, "Initial system matrix with variable elements {}", Logger::matrixToString(mVariableSystemMatrix));
			SPDLOG_LOGGER_INFO(mSLog, "Right side vector: {}", Logger::matrixToString(mRightSideVector));
		} else {
			if (mSwitches.size() < 1) {
				SPDLOG_LOGGER_INFO(mSLog, "System matrix: \n{}", mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][0]);
			}
			else {
				SPDLOG_LOGGER_INFO(mSLog, "Initial switch status: {:s}", mCurrentSwitchStatus.to_string());

				for (auto sys : mSwitchedMatrices) {
					SPDLOG_LOGGER_INFO(mSLog, "Switching System matrix {:s} \n{:s}",
					sys.first.to_string(), Logger::matrixToString(sys.second[0]));
				}
			}
			SPDLOG_LOGGER_INFO(mSLog, "Right side vector: \n{}", mRightSideVector);
		}
	}
}

template<typename VarType>
void MnaSolverDirect<VarType>::logLUTimes() {
	logFactorizationTime();
	logRecomputationTime();
	logSolveTime();
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
  	SPDLOG_LOGGER_INFO(mSLog, "Cumulative solve times: {:.12f}", solveSum);
	SPDLOG_LOGGER_INFO(mSLog, "Average solve time: {:.12f}", solveSum/static_cast<double>(mSolveTimes.size()));
	SPDLOG_LOGGER_INFO(mSLog, "Maximum solve time: {:.12f}", solveMax);
	SPDLOG_LOGGER_INFO(mSLog, "Number of solves: {:d}", mSolveTimes.size());
}


template <typename VarType>
void MnaSolverDirect<VarType>::logFactorizationTime()
{
	for (auto meas : mFactorizeTimes) {
		SPDLOG_LOGGER_INFO(mSLog, "LU factorization time: {:.12f}", meas);
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
		    SPDLOG_LOGGER_INFO(mSLog, "Cumulative refactorization times: {:.12f}",recompSum);
       		SPDLOG_LOGGER_INFO(mSLog, "Average refactorization time: {:.12f}", recompSum/((double)mRecomputationTimes.size()));
			SPDLOG_LOGGER_INFO(mSLog, "Maximum refactorization time: {:.12f}", recompMax);
       		SPDLOG_LOGGER_INFO(mSLog, "Number of refactorizations: {:d}", mRecomputationTimes.size());
	   }
}

template<typename VarType>
std::shared_ptr<DirectLinearSolver> MnaSolverDirect<VarType>::createDirectSolverImplementation(CPS::Logger::Log mSLog) {
	switch(this->mImplementationInUse)
	{
		case CPS::DirectLinearSolverImpl::DenseLU:
			return std::make_shared<DenseLUAdapter>(mSLog);
		case CPS::DirectLinearSolverImpl::SparseLU:
			return std::make_shared<SparseLUAdapter>(mSLog);
		#ifdef WITH_KLU
		case CPS::DirectLinearSolverImpl::KLU:
			return std::make_shared<KLUAdapter>(mSLog);
		#endif
		#ifdef WITH_CUDA
		case CPS::DirectLinearSolverImpl::CUDADense:
			return std::make_shared<GpuDenseAdapter>(mSLog);
		#ifdef WITH_CUDA_SPARSE
		case CPS::DirectLinearSolverImpl::CUDASparse:
			return std::make_shared<GpuSparseAdapter>(mSLog);
		#endif
		#ifdef WITH_MAGMA
		case CPS::DirectLinearSolverImpl::CUDAMagma:
			return std::make_shared<GpuMagmaAdapter>(mSLog);
		#endif
		#endif
		default:
			throw CPS::SystemError("unsupported linear solver implementation.");
	}
}

template <typename VarType>
void MnaSolverDirect<VarType>::setDirectLinearSolverImplementation(DirectLinearSolverImpl implementation) {
	this->mImplementationInUse = implementation;
}

template <typename VarType>
void MnaSolverDirect<VarType>::setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration& configuration) {
	this->mConfigurationInUse = configuration;
}

}

template class DPsim::MnaSolverDirect<Real>;
template class DPsim::MnaSolverDirect<Complex>;
