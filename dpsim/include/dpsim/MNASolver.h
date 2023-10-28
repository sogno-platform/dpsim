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

#include <dpsim/Config.h>
#include <dpsim/Solver.h>
#include <dpsim/DataLogger.h>
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/Solver/MNASyncGenInterface.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/SimPowerComp.h>

/* std::size_t is the largest data type. No container can store
 * more than std::size_t elements. Define the number of switches
 * as the log_2 of this value so that we end up with maximally
 * std::size_t matrices. The overhead of statically defining this
 * value should be minimal.
 **/
#define SWITCH_NUM sizeof(std::size_t)*8

namespace DPsim {
	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolver : public Solver {
	protected:
		// #### General simulation settings ####
		/// Simulation domain, which can be dynamic phasor (DP) or EMT
		CPS::Domain mDomain;
		/// Number of network and virtual nodes, single line equivalent
		UInt mNumNodes = 0;
		/// Number of network nodes, single line equivalent
		UInt mNumNetNodes = 0;
		/// Number of virtual nodes, single line equivalent
		UInt mNumVirtualNodes = 0;
		/// Number of network and virtual nodes, considering individual phases
		UInt mNumMatrixNodeIndices = 0;
		/// Number of network nodes, considering individual phases
		UInt mNumNetMatrixNodeIndices = 0;
		/// Number of virtual nodes, considering individual phases
		UInt mNumVirtualMatrixNodeIndices = 0;
		/// Number of nodes, excluding the primary frequency
		UInt mNumHarmMatrixNodeIndices = 0;
		/// Total number of network and virtual nodes, considering individual phases and additional frequencies
		UInt mNumTotalMatrixNodeIndices = 0;
		/// List of index pairs of varying matrix entries
		std::vector<std::pair<UInt, UInt>> mListVariableSystemMatrixEntries;

		/// System topology
		CPS::SystemTopology mSystem;
		/// List of simulation nodes
		typename CPS::SimNode<VarType>::List mNodes;

		// #### MNA specific attributes ####
		/// List of MNA components with static stamp into system matrix
		CPS::MNAInterface::List mMNAComponents;
		/// List of switches that stamp differently depending on their state
		/// and indicate the solver to choose a different system matrix
		CPS::MNASwitchInterface::List mSwitches;
		/// List of switches if they must be accessed as MNAInterface objects
		CPS::MNAInterface::List mMNAIntfSwitches;
		/// List of signal type components that do not directly interact with the MNA solver
		CPS::SimSignalComp::List mSimSignalComps;
		/// Current status of all switches encoded as bitset
		std::bitset<SWITCH_NUM> mCurrentSwitchStatus;
		/// List of synchronous generators that need iterate to solve the differential equations
		CPS::MNASyncGenInterface::List mSyncGen;

		/// Source vector of known quantities
		Matrix mRightSideVector;
		/// List of all right side vector contributions
		std::vector<const Matrix*> mRightVectorStamps;

		// #### MNA specific attributes related to harmonics / additional frequencies ####
		/// Source vector of known quantities
		std::vector<Matrix> mRightSideVectorHarm;

		// #### MNA specific attributes related to system recomputation
		/// Number of system matrix recomputations
		Int mNumRecomputations = 0;
		/// List of components that indicate the solver to recompute the system matrix
		/// depending on their state
		CPS::MNAVariableCompInterface::List mVariableComps;
		/// List of variable components if they must be accessed as MNAInterface objects
		CPS::MNAInterface::List mMNAIntfVariableComps;

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

		/// LU factorization measurements
		std::vector<Real> mFactorizeTimes;
		/// Right-hand side solution measurements
		std::vector<Real> mSolveTimes;
		/// LU refactorization measurements
		std::vector<Real> mRecomputationTimes;

		/// Constructor should not be called by users but by Simulation
		MnaSolver(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		/// Initialization of individual components
		void initializeComponents();
		/// Initialization of system matrices and source vector
		virtual void initializeSystem();
		/// Initialization of system matrices and source vector
		void initializeSystemWithParallelFrequencies();
		/// Initialization of system matrices and source vector
		void initializeSystemWithPrecomputedMatrices();
		/// Initialization of system matrices and source vector
		void initializeSystemWithVariableMatrix();
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
		virtual void createEmptySystemMatrix() = 0;
		/// Sets all entries in the matrix with the given switch index to zero
		virtual void switchedMatrixEmpty(std::size_t index) = 0;
		/// Sets all entries in the matrix with the given switch index and frequency index to zero
		virtual void switchedMatrixEmpty(std::size_t swIdx, Int freqIdx) = 0;
		/// Applies a component stamp to the matrix with the given switch index
		virtual void switchedMatrixStamp(std::size_t index, std::vector<std::shared_ptr<CPS::MNAInterface>>& comp) = 0;
		/// Applies a component and switch stamp to the matrix with the given switch index
		virtual void switchedMatrixStamp(std::size_t swIdx, Int freqIdx, CPS::MNAInterface::List& components, CPS::MNASwitchInterface::List& switches) { }
		/// Checks whether the status of variable MNA elements have changed
		Bool hasVariableComponentChanged();

		// #### Methods to implement for system recomputation over time ####
		/// Stamps components into the variable system matrix
		virtual void stampVariableSystemMatrix() = 0;
		/// Solves the system with variable system matrix
		virtual void solveWithSystemMatrixRecomputation(Real time, Int timeStepCount) = 0;
		/// Create a solve task for recomputation solver
		virtual std::shared_ptr<CPS::Task> createSolveTaskRecomp() = 0;

		/// Logging of system matrices and source vector
		virtual void logSystemMatrices() = 0;

		/// Create a solve task for this solver implementation
		virtual std::shared_ptr<CPS::Task> createSolveTask() = 0;
		/// Create a solve task for this solver implementation
		virtual std::shared_ptr<CPS::Task> createLogTask() = 0;
		/// Create a solve task for this solver implementation
		virtual std::shared_ptr<CPS::Task> createSolveTaskHarm(UInt freqIdx) = 0;

		// #### Scheduler Task Methods ####
		/// Solves system for single frequency
		virtual void solve(Real time, Int timeStepCount) = 0;
		/// Solves system for multiple frequencies
		virtual void solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx) = 0;
		/// Logs left and right vector
		virtual void log(Real time, Int timeStepCount) override;

	public:
		/// Solution vector of unknown quantities
		CPS::Attribute<Matrix>::Ptr mLeftSideVector;

		/// Solution vector of unknown quantities (parallel frequencies)
		std::vector<CPS::Attribute<Matrix>::Ptr> mLeftSideVectorHarm;

		/// Destructor
		virtual ~MnaSolver() {
			if (mSystemMatrixRecomputation)
				SPDLOG_LOGGER_INFO(mSLog, "Number of system matrix recomputations: {:}", mNumRecomputations);
		};

		/// Calls subroutines to set up everything that is required before simulation
		virtual void initialize() override;

		// #### Setter and Getter ####
		///
		virtual void setSystem(const CPS::SystemTopology &system) override;
		///
		Matrix& leftSideVector() { return **mLeftSideVector; }
		///
		Matrix& rightSideVector() { return mRightSideVector; }
		///
		virtual CPS::Task::List getTasks() override;

	};
}
