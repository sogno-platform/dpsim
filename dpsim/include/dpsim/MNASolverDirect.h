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
#include <dpsim/Definitions.h>
#include <dpsim/Solver.h>
#include <dpsim/SolverParametersMNA.h>
#include <dpsim/DataLogger.h>
#include <dpsim/DirectLinearSolver.h>
#include <dpsim/DirectLinearSolverConfiguration.h>
#include <dpsim/DenseLUAdapter.h>
#ifdef WITH_KLU
#include <dpsim/KLUAdapter.h>
#endif
#include <dpsim/SparseLUAdapter.h>
#ifdef WITH_CUDA
#include <dpsim/GpuDenseAdapter.h>
#ifdef WITH_CUDA_SPARSE
#include <dpsim/GpuSparseAdapter.h>
#endif
#ifdef WITH_MAGMA
#include <dpsim/GpuMagmaAdapter.h>
#endif
#endif
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim/MNASolver.h>


namespace DPsim {

	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolverDirect : public MnaSolver<VarType> {

	protected:
		// #### Data structures for precomputed switch matrices (optionally with parallel frequencies) ####
		/// Map of system matrices where the key is the bitset describing the switch states
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector<SparseMatrix> > mSwitchedMatrices;
		/// Map of direct linear solvers related to the system matrices
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector< std::shared_ptr< DirectLinearSolver> > > mDirectLinearSolvers;

		// #### Data structures for system recomputation over time ####
		/// System matrix including all static elements
		SparseMatrix mBaseSystemMatrix;
		/// System matrix including stamp of static and variable elements
		SparseMatrix mVariableSystemMatrix;
		/// LU factorization of variable system matrix
		std::shared_ptr<DirectLinearSolver> mDirectLinearSolverVariableSystemMatrix;
		/// LU factorization indicator
		CPS::DirectLinearSolverImpl mImplementationInUse;
		/// LU factorization configuration
		DirectLinearSolverConfiguration mConfigurationInUse;

		using Solver::mSolverParams;
		using MnaSolver<VarType>::mSwitches;
		using MnaSolver<VarType>::mMNAIntfSwitches;
		using MnaSolver<VarType>::mMNAComponents;
		using MnaSolver<VarType>::mVariableComps;
		using MnaSolver<VarType>::mMNAIntfVariableComps;
		using MnaSolver<VarType>::mRightSideVector;
		using MnaSolver<VarType>::mLeftSideVector;
		using MnaSolver<VarType>::mCurrentSwitchStatus;
		using MnaSolver<VarType>::mRightVectorStamps;
		using MnaSolver<VarType>::mNumNetNodes;
		using MnaSolver<VarType>::mNodes;
		using MnaSolver<VarType>::mIsInInitialization;
		using MnaSolver<VarType>::mRightSideVectorHarm;
		using MnaSolver<VarType>::mLeftSideVectorHarm;
		using MnaSolver<VarType>::mSLog;
		using MnaSolver<VarType>::hasVariableComponentChanged;
		using MnaSolver<VarType>::mNumRecomputations;
		using MnaSolver<VarType>::mSyncGen;
		using MnaSolver<VarType>::mFactorizeTimes;
		using MnaSolver<VarType>::mSolveTimes;
		using MnaSolver<VarType>::mRecomputationTimes;
		using MnaSolver<VarType>::mListVariableSystemMatrixEntries;

		// #### General
		/// Create system matrix
		void createEmptySystemMatrix() override;

		// #### Methods for precomputed switch matrices (optionally with parallel frequencies) ####
		/// Sets all entries in the matrix with the given switch index to zero
		void switchedMatrixEmpty(std::size_t index) override;
		/// Sets all entries in the matrix with the given switch index and frequency index to zero
		void switchedMatrixEmpty(std::size_t swIdx, Int freqIdx) override;
		/// Applies a component stamp to the matrix with the given switch index
		void switchedMatrixStamp(std::size_t index, std::vector<std::shared_ptr<CPS::MNAInterface>>& comp) override;

		// #### Methods for system recomputation over time ####
		/// Stamps components into the variable system matrix
		void stampVariableSystemMatrix() override;
		/// Solves the system with variable system matrix
		void solveWithSystemMatrixRecomputation(Real time, Int timeStepCount) override;
		/// Create a solve task for recomputation solver
		std::shared_ptr<CPS::Task> createSolveTaskRecomp() override;
		/// Recomputes systems matrix
		virtual void recomputeSystemMatrix(Real time);

		// #### Scheduler Task Methods ####
		/// Create a solve task for this solver implementation
		std::shared_ptr<CPS::Task> createSolveTask() override;
		/// Create a solve task for this solver implementation
		std::shared_ptr<CPS::Task> createLogTask() override;
		/// Create a solve task for this solver implementation
		std::shared_ptr<CPS::Task> createSolveTaskHarm(UInt freqIdx) override;
		/// Logging of system matrices and source vector
		void logSystemMatrices() override;
		/// Solves system for single frequency
		void solve(Real time, Int timeStepCount) override;
		/// Solves system for multiple frequencies
		void solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx) override;

		/// Logging of the right-hand-side solution time
		void logSolveTime();
		/// Logging of the LU factorization time
		void logFactorizationTime();
		/// Logging of the LU refactorization time
		void logRecomputationTime();

		/// Returns a pointer to an object of type DirectLinearSolver
		std::shared_ptr<DirectLinearSolver> createDirectSolverImplementation(CPS::Logger::Log mSLog);

	public:
		/// Constructor should not be called by users but by Simulation
		/// sovlerImpl: choose the most advanced solver implementation available by default
		MnaSolverDirect(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		/// Destructor
		virtual ~MnaSolverDirect() = default;

		/// Sets the linear solver to "implementation" and creates an object
		void setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl implementation);

		/// Sets the linear solver configuration
		void setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration& configuration);

		/// log LU decomposition times
		void logLUTimes() override;

		/// ### SynGen Interface ###
		int mIter = 0;
		
		/// Gets Solver Parameters for Modified Nodal Analysis (MNA) 
        std::shared_ptr<SolverParametersMNA> getMNAParameters() { return std::dynamic_pointer_cast<SolverParametersMNA>(mSolverParams); }


		// #### MNA Solver Tasks ####
		///
		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolverDirect<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {

				for (auto it : solver.mMNAComponents) {
					if (it->getRightVector()->get().size() != 0)
						mAttributeDependencies.push_back(it->getRightVector());
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->mVoltage);
				}
				mModifiedAttributes.push_back(solver.mLeftSideVector);
			}

			void execute(Real time, Int timeStepCount) {
				mSolver.solve(time, timeStepCount);
			}

		private:
			MnaSolverDirect<VarType>& mSolver;
		};

		///
		class SolveTaskHarm : public CPS::Task {
		public:
			SolveTaskHarm(MnaSolverDirect<VarType>& solver, UInt freqIdx) :
				Task(solver.mName + ".Solve"), mSolver(solver), mFreqIdx(freqIdx) {

				for (auto it : solver.mMNAComponents) {
					if (it->getRightVector()->get().size() != 0)
						mAttributeDependencies.push_back(it->getRightVector());
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->mVoltage);
				}
				for(auto leftVec : solver.mLeftSideVectorHarm) {
					mModifiedAttributes.push_back(leftVec);
				}
			}

			void execute(Real time, Int timeStepCount) {
				mSolver.solveWithHarmonics(time, timeStepCount, mFreqIdx);
			}

		private:
			MnaSolverDirect<VarType>& mSolver;
			UInt mFreqIdx;
		};

		///
		class SolveTaskRecomp : public CPS::Task {
		public:
			SolveTaskRecomp(MnaSolverDirect<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {

				for (auto it : solver.mMNAComponents) {
					if (it->getRightVector()->get().size() != 0)
						mAttributeDependencies.push_back(it->getRightVector());
				}
				for (auto it : solver.mMNAIntfVariableComps) {
					if (it->getRightVector()->get().size() != 0)
						mAttributeDependencies.push_back(it->getRightVector());
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->mVoltage);
				}
				mModifiedAttributes.push_back(solver.mLeftSideVector);
			}

			void execute(Real time, Int timeStepCount) {
				mSolver.solveWithSystemMatrixRecomputation(time, timeStepCount);
				mSolver.log(time, timeStepCount);
			}

		private:
			MnaSolverDirect<VarType>& mSolver;
		};

		///
		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolverDirect<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.mLeftSideVector);
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolverDirect<VarType>& mSolver;
		};
	};
}
