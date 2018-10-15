/** Simulation
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#include <cps/SignalComponent.h>
#include <cps/PowerComponent.h>

#define SWITCH_NUM 32

namespace DPsim {
	/// Solver class using Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolver : public Solver, public CPS::AttributeList {
	protected:
		// General simulation settings
		/// System time step is constant for MNA solver
		Real mTimeStep;
		/// Simulation domain, which can be dynamic phasor (DP) or EMT
		CPS::Domain mDomain;
		/// Number of nodes
		UInt mNumNodes = 0;
		/// Number of network nodes
		UInt mNumNetNodes = 0;
		/// Number of virtual nodes
		UInt mNumVirtualNodes = 0;
		/// Number of simulation nodes
		UInt mNumSimNodes = 0;
		/// Number of simulation network nodes
		UInt mNumNetSimNodes = 0;
		/// Number of simulation virtual nodes
		UInt mNumVirtualSimNodes = 0;
		/// Flag to activate power flow based initialization.
		/// If this is false, all voltages are initialized with zero.
		Bool mPowerflowInitialization;
		/// System list
		CPS::SystemTopology mSystem;
		///
		typename CPS::Node<VarType>::List mNodes;
		///
		CPS::MNAInterface::List mPowerComponents;
		///
		CPS::MNASwitchInterface::List mSwitches;
		///
		CPS::SignalComponent::List mSignalComponents;

		// #### MNA specific attributes ####
		/// System matrix A that is modified by matrix stamps
		std::bitset<SWITCH_NUM> mCurrentSwitchStatus;
		/// Temporary system matrix, i.e. for initialization
		Matrix mTmpSystemMatrix;
		/// LU decomposition of system matrix A
		CPS::LUFactorized mTmpLuFactorization;
		/// Source vector of known quantities
		Matrix mRightSideVector;
		/// Solution vector of unknown quantities
		Matrix mLeftSideVector;
		/// Switch to trigger steady-state initialization
		Bool mSteadyStateInit = false;
		/// Map of system matrices where the key is the bitset describing the switch states
		std::unordered_map< std::bitset<SWITCH_NUM>, Matrix > mSwitchedMatrices;
		/// Map of LU factorizations related to the system matrices
		std::unordered_map< std::bitset<SWITCH_NUM>, CPS::LUFactorized > mLuFactorizations;

		// #### Attributes related to switching ####
		/// Index of the next switching event
		UInt mSwitchTimeIndex = 0;
		/// Vector of switch times
		std::vector<SwitchConfiguration> mSwitchEvents;

		// #### Attributes related to logging ####
		/// Last simulation time step when log was updated
		Int mLastLogTimeStep = 0;
		/// Down sampling rate for log
		Int mDownSampleRate = 1;
		/// Simulation log level
		CPS::Logger::Level mLogLevel;
		/// Simulation logger
		CPS::Logger mLog;
		/// Left side vector logger
		DataLogger mLeftVectorLog;
		/// Right side vector logger
		DataLogger mRightVectorLog;
		/// Left side vector logger for initialization
		DataLogger mInitLeftVectorLog;
		/// Right side vector logger for initialization
		DataLogger mInitRightVectorLog;

		/// TODO: check that every system matrix has the same dimensions
		void initialize(CPS::SystemTopology system);
		/// Identify Nodes and PowerComponents and SignalComponents
		void identifyTopologyObjects();
		///
		void sortExecutionPriority();
		/// Assign simulation node index according to index in the vector.
		void assignSimNodes();
		/// Creates virtual nodes inside components.
		/// The MNA algorithm handles these nodes in the same way as network nodes.
		void createVirtualNodes();
		///
		void processSwitches();
		// TODO: check if this works with AC sources
		void steadyStateInitialization();
		/// Create left and right side vector
		void createEmptyVectors();
		/// Create system matrix
		void createEmptySystemMatrix();
		/// Solve system matrices
		void solve();
		///
		void updateSwitchStatus();
	public:
		/// This constructor should not be called by users.
		MnaSolver(String name,
			Real timeStep,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO,
			Bool steadyStateInit = false, Int downSampleRate = 1) :
			mTimeStep(timeStep),
			mDomain(domain),
			mSteadyStateInit(steadyStateInit),
			mDownSampleRate(downSampleRate),
			mLogLevel(logLevel),
			mLog(name + "_MNA", logLevel),
			mLeftVectorLog(name + "_LeftVector", logLevel != CPS::Logger::Level::NONE),
			mRightVectorLog(name + "_RightVector", logLevel != CPS::Logger::Level::NONE),
			mInitLeftVectorLog(name + "_InitLeftVector", logLevel != CPS::Logger::Level::NONE),
			mInitRightVectorLog(name + "_InitRightVector", logLevel != CPS::Logger::Level::NONE)
		{ }

		/// Constructor to be used in simulation examples.
		MnaSolver(String name, CPS::SystemTopology system,
			Real timeStep,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO,
			Bool steadyStateInit = false,
			Int downSampleRate = 1)
			: MnaSolver(name, timeStep, domain,
			logLevel, steadyStateInit, downSampleRate) {
			initialize(system);
		}

		///
		virtual ~MnaSolver() { };

		/// Solve system A * x = z for x and current time
		Real step(Real time);
		/// Log left and right vector values for each simulation step
		void log(Real time) {
			if (mDomain == CPS::Domain::EMT) {
				mLeftVectorLog.logEMTNodeValues(time, leftSideVector());
				mRightVectorLog.logEMTNodeValues(time, rightSideVector());
			}
			else {
				mLeftVectorLog.logPhasorNodeValues(time, leftSideVector());
				mRightVectorLog.logPhasorNodeValues(time, rightSideVector());
			}
		}
		// #### Getter ####
		Matrix& leftSideVector() { return mLeftSideVector; }
		Matrix& rightSideVector() { return mRightSideVector; }
		Matrix& systemMatrix() { return mTmpSystemMatrix; }

		CPS::Task::List getTasks();

		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolver<VarType>& solver) : mSolver(solver) {
				for (auto it : solver.mPowerComponents) {
					try {
						mAttributeDependencies.push_back(it->attribute("b"));
					} catch (CPS::InvalidAttributeException& e) {
						// component does not contribute to right side vector
					}
				}
				mModifiedAttributes.push_back(solver.attribute("x"));
			}

			String toString() const {
				return "MNASolver.Solve";
			}

			void execute(Real time, Int timeStepCount);

		private:
			MnaSolver<VarType>& mSolver;
		};
	};


}
