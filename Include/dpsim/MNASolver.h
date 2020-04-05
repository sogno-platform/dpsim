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

#include <dpsim/Solver.h>
#include <dpsim/DataLogger.h>
#include <cps/AttributeList.h>
#include <cps/Solver/MNASwitchInterface.h>
#include <cps/SimSignalComp.h>
#include <cps/SimPowerComp.h>

#define SWITCH_NUM 16

namespace DPsim {
	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolver : public Solver, public CPS::AttributeList {
	protected:
		// General simulation settings

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
		/// Flag to activate power flow based initialization.
		/// If this is false, all voltages are initialized with zero.
		Bool mPowerflowInitialization;
		/// System list
		CPS::SystemTopology mSystem;
		///
		typename CPS::SimNode<VarType>::List mNodes;
		///
		CPS::MNAInterface::List mMNAComponents;
		///
		CPS::MNASwitchInterface::List mSwitches;
		///
		CPS::SimSignalComp::List mSimSignalComps;

		// #### MNA specific attributes ####
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

		/// Map of system matrices where the key is the bitset describing the switch states
		std::unordered_map< std::bitset<SWITCH_NUM>, Matrix > mSwitchedMatrices;
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector<Matrix> > mSwitchedMatricesHarm;
		/// Map of LU factorizations related to the system matrices
		std::unordered_map< std::bitset<SWITCH_NUM>, CPS::LUFactorized > mLuFactorizations;
		std::unordered_map< std::bitset<SWITCH_NUM>, std::vector<CPS::LUFactorized> > mLuFactorizationsHarm;

		// #### Attributes related to switching ####
		/// Index of the next switching event
		UInt mSwitchTimeIndex = 0;
		/// Vector of switch times
		std::vector<SwitchConfiguration> mSwitchEvents;

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
		void initializeSystem();
		/// Identify Nodes and SimPowerComps and SimSignalComps
		void identifyTopologyObjects();
		/// Assign simulation node index according to index in the vector.
		void assignMatrixNodeIndices();
		/// Creates virtual nodes inside components.
		/// The MNA algorithm handles these nodes in the same way as network nodes.
		void createVirtualNodes();
		// TODO: check if this works with AC sources
		void steadyStateInitialization();
		/// Create left and right side vector
		void createEmptyVectors();
		/// Create system matrix
		void createEmptySystemMatrix();
		///
		void updateSwitchStatus();
		/// Logging of system matrices and source vector
		void logSystemMatrices();
	public:
		/// This constructor should not be called by users.
		MnaSolver(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		///
		virtual ~MnaSolver() { };

		///
		void setSystem(CPS::SystemTopology system);
		/// TODO: check that every system matrix has the same dimensions
		void initialize();
		/// Log left and right vector values for each simulation step
		void log(Real time);

		// #### Getter ####
		///
		Matrix& leftSideVector() { return mLeftSideVector; }
		///
		Matrix& rightSideVector() { return mRightSideVector; }
		///
		CPS::Task::List getTasks();
		///
		Matrix& systemMatrix() {
			return mSwitchedMatrices[mCurrentSwitchStatus];
		}

		///
		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolver<VarType>& solver, Bool steadyStateInit) :
				Task(solver.mName + ".Solve"), mSolver(solver), mSteadyStateInit(steadyStateInit) {
				for (auto it : solver.mMNAComponents) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0) {
						mAttributeDependencies.push_back(it->attribute("right_vector"));
					}
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->attribute("v"));
				}
				mModifiedAttributes.push_back(solver.attribute("left_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			MnaSolver<VarType>& mSolver;
			Bool mSteadyStateInit;
		};
		///
		class SolveTaskHarm : public CPS::Task {
		public:
			SolveTaskHarm(MnaSolver<VarType>& solver, Bool steadyStateInit, UInt freqIdx) :
				Task(solver.mName + ".Solve"), mSolver(solver), mSteadyStateInit(steadyStateInit), mFreqIdx(freqIdx) {
				for (auto it : solver.mMNAComponents) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0) {
						mAttributeDependencies.push_back(it->attribute("right_vector"));
					}
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->attribute("v"));
				}
				for(Int freq = 0; freq < solver.mSystem.mFrequencies.size(); freq++) {
					mModifiedAttributes.push_back(solver.attribute("left_vector_"+std::to_string(freq)));
				}
			}

			void execute(Real time, Int timeStepCount);

		private:
			MnaSolver<VarType>& mSolver;
			Bool mSteadyStateInit;
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

			void execute(Real time, Int timeStepCount);

		private:
			MnaSolver<VarType>& mSolver;
		};
	};
}
