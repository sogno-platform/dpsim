/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverEigenDense.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {


template <typename VarType>
MnaSolverEigenDense<VarType>::MnaSolverEigenDense(String name, CPS::Domain domain, CPS::Logger::Level logLevel) : MnaSolver<VarType>(name, domain, logLevel) {
}

template <typename VarType>
void MnaSolverEigenDense<VarType>::switchedMatrixEmpty(std::size_t index) {
	mSwitchedMatrices[std::bitset<SWITCH_NUM>(index)][0].setZero();
}

template <typename VarType>
void MnaSolverEigenDense<VarType>::switchedMatrixEmpty(std::size_t swIdx, Int freqIdx) {
	mSwitchedMatrices[std::bitset<SWITCH_NUM>(swIdx)][freqIdx].setZero();
}

template <typename VarType>
void MnaSolverEigenDense<VarType>::switchedMatrixStamp(std::size_t index, MNAInterface::List& components) {

	auto bit = std::bitset<SWITCH_NUM>(index);
	auto& sys = mSwitchedMatrices[bit][0];

	for (auto comp : components)
		comp->mnaApplySystemMatrixStamp(sys);

	for (UInt i = 0; i < mSwitches.size(); ++i)
		mSwitches[i]->mnaApplySwitchSystemMatrixStamp(bit[i], sys, 0);

	// Compute LU-factorization for system matrix
	mLuFactorizations[bit][0].compute(sys);
}

template <typename VarType>
void MnaSolverEigenDense<VarType>::switchedMatrixStamp(std::size_t swIdx, Int freqIdx,
	MNAInterface::List& components, MNASwitchInterface::List& switches) {

	auto bit = std::bitset<SWITCH_NUM>(swIdx);
	auto& sys = mSwitchedMatrices[bit][freqIdx];

	for (auto comp : components)
		comp->mnaApplySystemMatrixStampHarm(sys, freqIdx);

	for (UInt i = 0; i < switches.size(); ++i)
		switches[i]->mnaApplySwitchSystemMatrixStamp(bit[i], sys, freqIdx);

	mLuFactorizations[bit][freqIdx].compute(sys);
}

template <>
void MnaSolverEigenDense<Real>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
		auto bit = std::bitset<SWITCH_NUM>(i);
		mSwitchedMatrices[bit].push_back(Matrix::Zero(mNumMatrixNodeIndices, mNumMatrixNodeIndices));
		mLuFactorizations[bit].push_back(Eigen::PartialPivLU<Matrix>());
	}
	mBaseSystemMatrix = Matrix::Zero(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
}

template <>
void MnaSolverEigenDense<Complex>::createEmptySystemMatrix() {
	if (mSwitches.size() > SWITCH_NUM)
		throw SystemError("Too many Switches.");

	if (mFrequencyParallel) {
		for (UInt i = 0; i < std::pow(2,mSwitches.size()); ++i) {
			for(Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
				auto bit = std::bitset<SWITCH_NUM>(i);
				mSwitchedMatrices[bit].push_back(Matrix::Zero(2*(mNumMatrixNodeIndices), 2*(mNumMatrixNodeIndices)));
				mLuFactorizations[bit].push_back(Eigen::PartialPivLU<Matrix>());
			}
		}
	}
	else {
		for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
			auto bit = std::bitset<SWITCH_NUM>(i);
			mSwitchedMatrices[bit].push_back(Matrix::Zero(2*(mNumTotalMatrixNodeIndices), 2*(mNumTotalMatrixNodeIndices)));
			mLuFactorizations[bit].push_back(Eigen::PartialPivLU<Matrix>());
		}
		mBaseSystemMatrix = Matrix::Zero(2 * (mNumTotalMatrixNodeIndices), 2 * (mNumTotalMatrixNodeIndices));
	}
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverEigenDense<VarType>::createSolveTask()
{
	return std::make_shared<MnaSolverEigenDense<VarType>::SolveTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverEigenDense<VarType>::createSolveTaskHarm(UInt freqIdx)
{
	return std::make_shared<MnaSolverEigenDense<VarType>::SolveTaskHarm>(*this, freqIdx);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverEigenDense<VarType>::createLogTask()
{
	return std::make_shared<MnaSolverEigenDense<VarType>::LogTask>(*this);
}

template <typename VarType>
void MnaSolverEigenDense<VarType>::solve(Real time, Int timeStepCount) {
	// Reset source vector
	mRightSideVector.setZero();

	// Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVector += *stamp;

	if (!mIsInInitialization)
		MnaSolver<VarType>::updateSwitchStatus();

	if (mSwitchedMatrices.size() > 0)
		mLeftSideVector = mLuFactorizations[mCurrentSwitchStatus][0].solve(mRightSideVector);

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
		mNodes[nodeIdx]->mnaUpdateVoltage(mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverEigenDense<VarType>::solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx) {
	mRightSideVectorHarm[freqIdx].setZero();

	// Sum of right side vectors (computed by the components' pre-step tasks)
	for (auto stamp : mRightVectorStamps)
		mRightSideVectorHarm[freqIdx] += stamp->col(freqIdx);

	mLeftSideVectorHarm[freqIdx] =	mLuFactorizations[mCurrentSwitchStatus][freqIdx].solve(mRightSideVectorHarm[freqIdx]);
}

template <typename VarType>
void MnaSolverEigenDense<VarType>::logSystemMatrices() {
	if (mFrequencyParallel) {
		for (UInt i = 0; i < mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)].size(); ++i) {
			mSLog->info("System matrix for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][i]));
			//mSLog->info("LU decomposition for frequency: {:d} \n{:s}", i,
			//	Logger::matrixToString(mLuFactorizations[std::bitset<SWITCH_NUM>(0)][i].matrixLU()));
		}

		for (UInt i = 0; i < mRightSideVectorHarm.size(); ++i)
			mSLog->info("Right side vector for frequency: {:d} \n{:s}", i,
				Logger::matrixToString(mRightSideVectorHarm[i]));

	}
	else {
		if (mSwitches.size() < 1) {
			mSLog->info("System matrix: \n{}", mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][0]);
			//mSLog->info("LU decomposition: \n{}",	mLuFactorizations[std::bitset<SWITCH_NUM>(0)].matrixLU());
		}
		else {
			mSLog->info("Initial switch status: {:s}", mCurrentSwitchStatus.to_string());

			for (auto sys : mSwitchedMatrices) {
				mSLog->info("Switching System matrix {:s} \n{:s}",
					sys.first.to_string(), Logger::matrixToString(sys.second[0]));
				//mSLog->info("LU Factorization for System Matrix {:s} \n{:s}",
				//	sys.first.to_string(), Logger::matrixToString(mLuFactorizations[sys.first].matrixLU()));
			}
		}
		mSLog->info("Right side vector: \n{}", mRightSideVector);
	}
}
}

template class DPsim::MnaSolverEigenDense<Real>;
template class DPsim::MnaSolverEigenDense<Complex>;
