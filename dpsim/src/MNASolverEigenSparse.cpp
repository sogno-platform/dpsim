/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverEigenSparse.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {


template <typename VarType>
MnaSolverEigenSparse<VarType>::MnaSolverEigenSparse(String name, CPS::Domain domain, CPS::Logger::Level logLevel) :	MnaSolver<VarType>(name, domain, logLevel) {
}


template <typename VarType>
void MnaSolverEigenSparse<VarType>::switchedMatrixEmpty(std::size_t index) {
	mSwitchedMatrices[std::bitset<SWITCH_NUM>(index)][0].setZero();
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::switchedMatrixEmpty(std::size_t swIdx, Int freqIdx) {
	mSwitchedMatrices[std::bitset<SWITCH_NUM>(swIdx)][freqIdx].setZero();
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::switchedMatrixStamp(std::size_t index, std::vector<std::shared_ptr<CPS::MNAInterface>>& comp)
{
	auto bit = std::bitset<SWITCH_NUM>(index);
	auto& sys = mSwitchedMatrices[bit][0];
	for (auto comp : comp) {
		comp->mnaApplySystemMatrixStamp(sys);
	}
	for (UInt i = 0; i < mSwitches.size(); ++i)
		mSwitches[i]->mnaApplySwitchSystemMatrixStamp(bit[i], sys, 0);

	// Compute LU-factorization for system matrix
	mLuFactorizations[bit][0]->analyzePattern(sys);
	mLuFactorizations[bit][0]->factorize(sys);
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::stampVariableSystemMatrix() {

	mSLog->info("Number of variable Elements: {}"
				"\nNumber of MNA components: {}",
				mVariableComps.size(),
				mMNAComponents.size());

	// Build base matrix with only static elements
	mBaseSystemMatrix.setZero();
	for (auto statElem : mMNAComponents)
		statElem->mnaApplySystemMatrixStamp(mBaseSystemMatrix);
	mSLog->info("Base matrix with only static elements: {}", Logger::matrixToString(mBaseSystemMatrix));
	mSLog->flush();
	
	// Use matrix with only static elements as basis for variable system matrix
	mVariableSystemMatrix = mBaseSystemMatrix;

	// Now stamp switches into matrix
	mSLog->info("Stamping switches");
	for (auto sw : mSwitches)
		sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	// Now stamp initial state of variable elements into matrix
	mSLog->info("Stamping variable elements");
	for (auto varElem : mMNAIntfVariableComps)
		varElem->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	mSLog->info("Initial system matrix with variable elements {}", Logger::matrixToString(mVariableSystemMatrix));
	mSLog->flush();

	// Calculate factorization of current matrix
	mLuFactorizationVariableSystemMatrix.analyzePattern(mVariableSystemMatrix);
	mLuFactorizationVariableSystemMatrix.factorize(mVariableSystemMatrix);
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::solveWithSystemMatrixRecomputation(Real time, Int timeStepCount) {
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
	**mLeftSideVector = mLuFactorizationVariableSystemMatrix.solve(mRightSideVector);

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
		mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::recomputeSystemMatrix(Real time) {
	// Start from base matrix
	mVariableSystemMatrix = mBaseSystemMatrix;

	// Now stamp switches into matrix
	for (auto sw : mSwitches)
		sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	// Now stamp variable elements into matrix
	for (auto comp : mMNAIntfVariableComps)
		comp->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

	// Refactorization of matrix assuming that structure remained
	// constant by omitting analyzePattern
	mLuFactorizationVariableSystemMatrix.factorize(mVariableSystemMatrix);
	++mNumRecomputations;
}

template <>
void MnaSolverEigenSparse<Real>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	if (mSystemMatrixRecomputation) {
		mBaseSystemMatrix = SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
		mVariableSystemMatrix = SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
	} else {
		for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++){
			auto bit = std::bitset<SWITCH_NUM>(i);
			mSwitchedMatrices[bit].push_back(SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices));
			mLuFactorizations[bit].push_back(std::make_shared<LUFactorizedSparse>());
		}
	}
}

template <>
void MnaSolverEigenSparse<Complex>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	if (mFrequencyParallel) {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); ++i) {
			for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
				auto bit = std::bitset<SWITCH_NUM>(i);
				mSwitchedMatrices[bit].push_back(SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices)));
				mLuFactorizations[bit].push_back(std::make_shared<LUFactorizedSparse>());
			}
		}
	} else if (mSystemMatrixRecomputation) {
		mBaseSystemMatrix = SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices));
		mVariableSystemMatrix = SparseMatrix(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices));
	} else {
		for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
			auto bit = std::bitset<SWITCH_NUM>(i);
			mSwitchedMatrices[bit].push_back(SparseMatrix(2*(mNumTotalMatrixNodeIndices), 2*(mNumTotalMatrixNodeIndices)));
			mLuFactorizations[bit].push_back(std::make_shared<LUFactorizedSparse>());
		}
	}
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverEigenSparse<VarType>::createSolveTask()
{
	return std::make_shared<MnaSolverEigenSparse<VarType>::SolveTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverEigenSparse<VarType>::createSolveTaskRecomp()
{
	return std::make_shared<MnaSolverEigenSparse<VarType>::SolveTaskRecomp>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverEigenSparse<VarType>::createSolveTaskHarm(UInt freqIdx)
{
	return std::make_shared<MnaSolverEigenSparse<VarType>::SolveTaskHarm>(*this, freqIdx);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverEigenSparse<VarType>::createLogTask()
{
	return std::make_shared<MnaSolverEigenSparse<VarType>::LogTask>(*this);
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::solve(Real time, Int timeStepCount) {
	mIter = 0;
	if 	(mSyncGen.size()==0) {
		// Reset source vector
		mRightSideVector.setZero();

		// Add together the right side vector (computed by the components'
		// pre-step tasks)
		for (auto stamp : mRightVectorStamps)
			mRightSideVector += *stamp;

		if (!mIsInInitialization)
			MnaSolver<VarType>::updateSwitchStatus();

		if (mSwitchedMatrices.size() > 0)
			**mLeftSideVector = mLuFactorizations[mCurrentSwitchStatus][0]->solve(mRightSideVector);
			
	} else {
		// if there is iterative syncGens, then it is necessary to iterate
		bool iterate = true;
		while (iterate) {
			// 
			mIter = mIter + 1;

			// Reset source vector
			mRightSideVector.setZero();

			if (!mIsInInitialization)
				MnaSolver<VarType>::updateSwitchStatus();

			for (auto syncGen : mSyncGen)
				syncGen->correctorStep();

			// Add together the right side vector (computed by the components'
			// pre-step tasks)
			for (auto stamp : mRightVectorStamps)
				mRightSideVector += *stamp;

			if (mSwitchedMatrices.size() > 0)
				**mLeftSideVector = mLuFactorizations[mCurrentSwitchStatus][0]->solve(mRightSideVector);

			for (auto syncGen : mSyncGen)
				//update voltages
				syncGen->updateVoltage(**mLeftSideVector);

			// check if there is sync generators that need iterate
			int count=0; 
			for (auto syncGen : mSyncGen) {
				if (syncGen->checkVoltageDifference())
					count = count+1;
			}

			/*
			for (auto sys : mSwitchedMatrices) {
				if (mCurrentSwitchStatus == sys.first) {
					//std::cout <<  Logger::matrixToString(sys.second[0]) << std::endl;
					//std::cout << "\n\n\nmRightSideVector: \n" << mRightSideVector << std::endl;
					//std::cout << "\n\n\nA * **mLeftSideVector: \n" << sys.second[0] *  **mLeftSideVector << std::endl;
					auto residual = mRightSideVector - sys.second[0] * **mLeftSideVector;
					auto max_residual = residual.cwiseAbs().maxCoeff();
					if ( mIter >= mMaxIterations) {
						iterate=false;
						if (max_residual > 1e-11)  {
							// std::cout << "\n\n\nResidual: \n" << mRightSideVector - sys.second[0] * **mLeftSideVector << std::endl;
							std::cout << "max = " << max_residual << std::endl;
							std::cout << "Iter: " << mIter << std::endl;
						}
						mIter = 0;
					}
					else {
						if (max_residual < 1e-11)  {
							std::cout << "Iter: " << mIter << std::endl;
							mIter = 0;
							iterate = false;
						}
						else {
							iterate = true;
						}
					}

				}
			}
			*/
			if (count==0)  {
				iterate=false;
			}	
		}
	}

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
		mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx) {
	mRightSideVectorHarm[freqIdx].setZero();

	// Sum of right side vectors (computed by the components' pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVectorHarm[freqIdx] += stamp->col(freqIdx);

	**mLeftSideVectorHarm[freqIdx] = mLuFactorizations[mCurrentSwitchStatus][freqIdx]->solve(mRightSideVectorHarm[freqIdx]);
}

template <typename VarType>
void MnaSolverEigenSparse<VarType>::logSystemMatrices() {
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
}

template class DPsim::MnaSolverEigenSparse<Real>;
template class DPsim::MnaSolverEigenSparse<Complex>;
