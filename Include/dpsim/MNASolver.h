/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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

#include <dpsim/Config.h>
#include <dpsim/Solver.h>
#include <dpsim/DataLogger.h>
#include <cps/AttributeList.h>
#include <cps/Solver/MNASwitchInterface.h>
#include <cps/Solver/MNAVariableCompInterface.h>
#include <cps/SimSignalComp.h>
#include <cps/SimPowerComp.h>

/* std::size_t is the largest data type. No container can store
 * more than std::size_t elements. Define the number of switches
 * as the log_2 of this value so that we end up with maximally
 * std::size_t matrices. The overhead of statically defining this
 * value should be minimal.
 **/
#define SWITCH_NUM sizeof(std::size_t)*8

#ifdef WITH_SPARSE
#define MAT_TYPE SparseMatrix
#else
#define MAT_TYPE Matrix
#endif

namespace DPsim {
	/// \brief The implementations of the MNA solvers MnaSolver can support.
	///
	enum MnaSolverImpl {
		EigenDense,
		EigenSparse,
		CUDADense,
		CUDASparse,
	};

	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolver : public Solver, public CPS::AttributeList {
	protected:
		// #### General simulation settings ####
		/// Simulation domain, which can be dynamic phasor (DP) or EMT
		CPS::Domain mDomain;
		/// Number of nodes
		UInt mNumNodes = 0;
		/// Number of network nodes
		UInt mNumNetNodes = 0;
		/// Number of virtual nodes
		UInt mNumVirtualNodes = 0;
		/// Number of simulation nodes
		UInt mNumMatrixNodeIndices = 0;
		/// Number of simulation network nodes
		UInt mNumNetMatrixNodeIndices = 0;
		/// Number of simulation virtual nodes
		UInt mNumVirtualMatrixNodeIndices = 0;
		/// Number of harmonic nodes
		UInt mNumHarmMatrixNodeIndices = 0;
		/// System list
		CPS::SystemTopology mSystem;
		/// List of simulation nodes
		typename CPS::SimNode<VarType>::List mNodes;

		/// MNA implementations supported by this compilation
		static const std::vector<MnaSolverImpl> mSupportedSolverImpls;
		/// MNA implementation chosen for this instance
		MnaSolverImpl mSolverImpl;

		// #### MNA specific attributes ####
		/// List of MNA components with static stamp into system matrix
		CPS::MNAInterface::List mMNAComponents;
		/// List of switches that stamp differently depending on their state
		/// and indicate the solver to choose a different system matrix
		CPS::MNASwitchInterface::List mSwitches;
		/// List of MNA components with SwitchInterface
		CPS::MNAInterface::List mMNAIntfSwitches;
		/// List of components that indicate the solver to recompute the system matrix
		/// depending on their state
		CPS::MNAVariableCompInterface::List mVariableComps;
		/// List of MNA components with VariableCompInterface
		CPS::MNAInterface::List mMNAIntfVariableComps;
		/// List of signal type components that do not directly interact
		/// with the MNA solver
		CPS::SimSignalComp::List mSimSignalComps;
		/// System matrix A that is modified by matrix stamps
		std::bitset<SWITCH_NUM> mCurrentSwitchStatus;
		/// Source vector of known quantities
		Matrix mRightSideVector;
		std::vector<Matrix> mRightSideVectorHarm;
		/// List of all right side vector contributions
		std::vector<const Matrix*> mRightVectorStamps;
		/// Solution vector of unknown quantities
		Matrix mLeftSideVector;
		std::vector<Matrix> mLeftSideVectorHarm;
		std::vector< CPS::Attribute<Matrix>::Ptr > mLeftVectorHarmAttributes;

		/// Base matrix that includes all static MNA elements to speed up recomputation
		MAT_TYPE mBaseSystemMatrix;
		/// Map of system matrices where the key is the bitset describing the switch states
		std::unordered_map< std::bitset<SWITCH_NUM>, MAT_TYPE > mSwitchedMatrices;
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector<Matrix> > mSwitchedMatricesHarm;
#ifdef WITH_SPARSE
		/// Map of LU factorizations related to the system matrices
		std::unordered_map< std::bitset<SWITCH_NUM>, CPS::LUFactorizedSparse > mLuFactorizations;
#else
		std::unordered_map< std::bitset<SWITCH_NUM>, CPS::LUFactorized > mLuFactorizations;
#endif
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector<CPS::LUFactorized> > mLuFactorizationsHarm;

		// #### Attributes related to switching ####
		/// Index of the next switching event
		UInt mSwitchTimeIndex = 0;
		/// Vector of switch times
		std::vector<SwitchConfiguration> mSwitchEvents;
		/// Collects the status of switches to select correct system matrix
		void updateSwitchStatus();

		// #### Attributes related to logging ####
		/// Last simulation time step when log was updated
		Int mLastLogTimeStep = 0;
		/// Left side vector logger
		std::shared_ptr<DataLogger> mLeftVectorLog;
		/// Right side vector logger
		std::shared_ptr<DataLogger> mRightVectorLog;

		/// Initialization of individual components
		void initializeComponents();
		/// Initialization of system matrices and source vector
		virtual void initializeSystem();
		/// Initialization of system matrices and source vector
		void initializeSystemWithParallelFrequencies();
		/// Initialization of system matrices and source vector
		void initializeSystemWithPrecomputedMatrices();
		/// Identify Nodes and SimPowerComps and SimSignalComps
		void identifyTopologyObjects();
		/// Assign simulation node index according to index in the vector.
		void assignMatrixNodeIndices();
		/// Collects virtual nodes inside components.
		/// The MNA algorithm handles these nodes in the same way as network nodes.
		void collectVirtualNodes();
		// TODO: check if this works with AC sources
		void steadyStateInitialization();
		/// Create left and right side vector
		void createEmptyVectors();
		/// Create system matrix
		void createEmptySystemMatrix();
		/// Logging of system matrices and source vector
		void logSystemMatrices();

		// #### Scheduler Task Methods ####
		/// Solves system for single frequency
		virtual void solve(Real time, Int timeStepCount);
		/// Solves system for multiple frequencies
		void solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx);
		/// Logs left and right vector
		void log(Real time, Int timeStepCount);

	public:
		/// Constructor should not be called by users but by Simulation
		/// sovlerImpl: choose the most advanced solver implementation available by default
		MnaSolver(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info,
			MnaSolverImpl solverImpl = *mSupportedSolverImpls.end());

		/// Destructor
		virtual ~MnaSolver() { };

		/// Calls subroutines to set up everything that is required before simulation
		void initialize();

		// #### Setter and Getter ####
		///
		void setSystem(const CPS::SystemTopology &system);
		///
		Matrix& leftSideVector() { return mLeftSideVector; }
		///
		Matrix& rightSideVector() { return mRightSideVector; }
		///
		virtual CPS::Task::List getTasks();
		///
		MAT_TYPE& systemMatrix() {
			return mSwitchedMatrices[mCurrentSwitchStatus];
		}

		// #### MNA Solver Tasks ####
		///
		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolver<VarType>& solver) :
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
			MnaSolver<VarType>& mSolver;
		};

		///
		class SolveTaskHarm : public CPS::Task {
		public:
			SolveTaskHarm(MnaSolver<VarType>& solver, UInt freqIdx) :
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
			MnaSolver<VarType>& mSolver;
			UInt mFreqIdx;
		};

		///
		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolver<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.attribute("left_vector"));
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolver<VarType>& mSolver;
		};
	};
}
