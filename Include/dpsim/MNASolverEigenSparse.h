/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <bitset>
#include <memory>

#include <dpsim/Config.h>
#include <dpsim/Solver.h>
#include <dpsim/DataLogger.h>
#include <cps/AttributeList.h>
#include <cps/Solver/MNASwitchInterface.h>
#include <cps/Solver/MNAVariableCompInterface.h>
#include <cps/SimSignalComp.h>
#include <cps/SimPowerComp.h>
#include <dpsim/MNASolver.h>


namespace DPsim {

	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolverEigenSparse : public MnaSolver<VarType> {
	protected:
		/// Base matrix that includes all static MNA elements to speed up recomputation
		SparseMatrix mBaseSystemMatrix;
		/// Map of system matrices where the key is the bitset describing the switch states
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector<SparseMatrix> > mSwitchedMatrices;
		/// Map of LU factorizations related to the system matrices
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector< std::shared_ptr< CPS::LUFactorizedSparse> > > mLuFactorizations;

		using MnaSolver<VarType>::mSwitches;
		using MnaSolver<VarType>::mRightSideVector;
		using MnaSolver<VarType>::mLeftSideVector;
		using MnaSolver<VarType>::mCurrentSwitchStatus;
		using MnaSolver<VarType>::mRightVectorStamps;
		using MnaSolver<VarType>::mNumNetNodes;
		using MnaSolver<VarType>::mNodes;
		using MnaSolver<VarType>::mIsInInitialization;
		using MnaSolver<VarType>::mRightSideVectorHarm;
		using MnaSolver<VarType>::mLeftSideVectorHarm;
		using MnaSolver<VarType>::mFrequencyParallel;
		using MnaSolver<VarType>::mSLog;


		/// Sets all entries in the matrix with the given switch index to zero
		virtual void switchedMatrixEmpty(std::size_t index) override;
		/// Sets all entries in the matrix with the given switch index and frequency index to zero
		virtual void switchedMatrixEmpty(std::size_t swIdx, Int freqIdx) override;
		/// Create system matrix
		virtual void createEmptySystemMatrix() override;
		/// Applies a component stamp to the matrix with the given switch index
		virtual void switchedMatrixStamp(std::size_t index, std::vector<std::shared_ptr<CPS::MNAInterface>>& comp) override;
		/// Create a solve task for this solver implementation
		virtual std::shared_ptr<CPS::Task> createSolveTask() override;
		/// Create a solve task for this solver implementation
		virtual std::shared_ptr<CPS::Task> createLogTask() override;
		/// Create a solve task for this solver implementation
		virtual std::shared_ptr<CPS::Task> createSolveTaskHarm(UInt freqIdx) override;
		/// Logging of system matrices and source vector
		virtual void logSystemMatrices() override;

		// #### Scheduler Task Methods ####
		/// Solves system for single frequency
		virtual void solve(Real time, Int timeStepCount) override;
		/// Solves system for multiple frequencies
		virtual void solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx) override;

	public:
		/// Constructor should not be called by users but by Simulation
		/// sovlerImpl: choose the most advanced solver implementation available by default
		MnaSolverEigenSparse(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		/// Destructor
		virtual ~MnaSolverEigenSparse() { };

		// #### MNA Solver Tasks ####
		///
		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolverEigenSparse<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {

				for (auto it : solver.mMNAComponents) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0)
						mAttributeDependencies.push_back(it->attribute("right_vector"));
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->attribute("v"));
				}
				mModifiedAttributes.push_back(solver.attribute("left_vector"));
			}

			void execute(Real time, Int timeStepCount) { mSolver.solve(time, timeStepCount); }

		private:
			MnaSolverEigenSparse<VarType>& mSolver;
		};

		///
		class SolveTaskHarm : public CPS::Task {
		public:
			SolveTaskHarm(MnaSolverEigenSparse<VarType>& solver, UInt freqIdx) :
				Task(solver.mName + ".Solve"), mSolver(solver), mFreqIdx(freqIdx) {

				for (auto it : solver.mMNAComponents) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0)
						mAttributeDependencies.push_back(it->attribute("right_vector"));
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->attribute("v"));
				}
				for(Int freq = 0; freq < solver.mSystem.mFrequencies.size(); ++freq) {
					mModifiedAttributes.push_back(solver.attribute("left_vector_"+std::to_string(freq)));
				}
			}

			void execute(Real time, Int timeStepCount) { mSolver.solveWithHarmonics(time, timeStepCount, mFreqIdx); }

		private:
			MnaSolverEigenSparse<VarType>& mSolver;
			UInt mFreqIdx;
		};

		///
		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolverEigenSparse<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.attribute("left_vector"));
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolverEigenSparse<VarType>& mSolver;
		};
	};
}
