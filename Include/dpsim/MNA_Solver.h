/** Simulation
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "Solver.h"
#ifdef WITH_CIM
#include "cps/CIM/Reader.h"
#endif /* WITH_CIM */

using namespace CPS;

namespace DPsim {
	/// Simulation class which uses Modified Nodal Analysis (MNA).
	template <typename VarType>
	class MnaSolver : public Solver {
	protected:
		// General simulation settings
		/// System time step is constant for MNA solver
		Real mTimeStep;
		/// Simulation domain, which can be dynamic phasor (DP) or EMT
		Domain mDomain;
		/// Number of nodes
		UInt mNumNodes = 0;
		/// Number of nodes
		UInt mNumRealNodes = 0;
		/// Number of nodes
		UInt mNumVirtualNodes = 0;
		/// Flag to activate power flow based initialization.
		/// If this is false, all voltages are initialized with zero.
		Bool mPowerflowInitialization;
		/// System list
		SystemTopology mSystem;
		/// 
		typename Node<VarType>::List mNodes;
		/// 
		typename PowerComponent<VarType>::List mPowerComponents;
		/// 
		SignalComponent::List mSignalComponents;
		
		// #### MNA specific attributes ####
		/// System matrix A that is modified by matrix stamps
		UInt mSystemIndex = 0;
		/// System matrices list for swtiching events
		std::vector<Matrix> mSystemMatrices;
		/// LU decomposition of system matrix A
		std::vector<LUFactorized> mLuFactorizations;
		/// Vector of known quantities
		Matrix mRightSideVector;
		/// Vector of unknown quantities
		Matrix mLeftSideVector;
		/// Switch to trigger steady-state initialization
		Bool mSteadyStateInit = false;

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
		Logger::Level mLogLevel;
		/// Simulation logger
		Logger mLog;
		/// Left side vector logger
		Logger mLeftVectorLog;
		/// Right side vector logger
		Logger mRightVectorLog;

		/// TODO: check that every system matrix has the same dimensions
		void initialize(SystemTopology system) {
			mLog.Log(Logger::Level::INFO) << "#### Start Initialization ####" << std::endl;
			mSystem = system;

			// We need to differentiate between power and signal components and
			// ground nodes should be ignored.
			IdentifyTopologyObjects();

			// These steps complete the network information.
			assignSimNodes();		
			createVirtualNodes();

			// For the power components the step order should not be important
			// but signal components need to be executed following the connections.
			sortExecutionPriority();

			// The system topology is prepared and we create the MNA matrices.
			createEmptyVectors();
			createEmptySystemMatrix();

			// TODO: Move to base solver class?
			// This intialization according to power flow information is not MNA specific.
			mLog.Log(Logger::Level::INFO) << "Initialize power flow" << std::endl;
			for (auto comp : mPowerComponents)
				comp->initializePowerflow(mSystem.mSystemFrequency);

			// This steady state initialization is MNA specific and runs a simulation 
			// before the actual simulation executed by the user.
			if (mSteadyStateInit && mDomain == Domain::DP) {
				mLog.Log(Logger::Level::INFO) << "Run steady-state initialization." << std::endl;
				steadyStateInitialization();
			}

			// Now, initialize the components for the actual simulation.			
			for (auto comp : mSignalComponents) {
				comp->initialize();
			}
			for (auto comp : mPowerComponents) {
				comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep);
				comp->mnaApplyRightSideVectorStamp(mRightSideVector);
				comp->mnaApplySystemMatrixStamp(mSystemMatrices[0]);
			}
						
			// Compute LU-factorization for system matrix
			mLuFactorizations.push_back(Eigen::PartialPivLU<Matrix>(mSystemMatrices[0]));
		}

		/// Identify Nodes and PowerComponents and SignalComponents
		void IdentifyTopologyObjects() {			
			for (auto baseNode : mSystem.mNodes) {	
				// Add nodes to the list and ignore ground nodes.
				if (!baseNode->isGround()) {			
					auto node = std::dynamic_pointer_cast< Node<VarType> >(baseNode);
					mNodes.push_back( node );	
				}
			}
			
			for (UInt i = 0; i < mNodes.size(); i++) {
				mLog.Log(Logger::Level::INFO) << "Found node " << mNodes[i]->getName() << std::endl;
			}

			for (auto comp : mSystem.mComponents) {
				if ( typename PowerComponent<VarType>::Ptr powercomp = std::dynamic_pointer_cast< PowerComponent<VarType> >(comp) ) {
					mPowerComponents.push_back(powercomp);		
				}
				else if ( SignalComponent::Ptr signalcomp = std::dynamic_pointer_cast< SignalComponent >(comp) )	{	
					mSignalComponents.push_back(signalcomp);	
				}
			}
		}

		void sortExecutionPriority() {			
			// Sort SignalComponents according to execution priority.
			// Components with a higher priority number should come first.
			std::sort(mSignalComponents.begin(), mSignalComponents.end(), [](const auto& lhs, const auto& rhs) {
				return lhs->getPriority() > rhs->getPriority();
			});
		}

		/// Assign simulation node index according to index in the vector.
		void assignSimNodes() {			
			UInt simNodeIdx = -1;
			for (UInt idx = 0; idx < mNodes.size(); idx++) {
				mNodes[idx]->getSimNodes()[0] = ++simNodeIdx;
				if (mNodes[idx]->getPhaseType() == PhaseType::ABC) {
					mNodes[idx]->getSimNodes()[1] = ++simNodeIdx;
					mNodes[idx]->getSimNodes()[2] = ++simNodeIdx;
				}
			}	

			// Total number of network nodes is simNodeIdx + 1
			mNumRealNodes = simNodeIdx + 1;	

			mLog.Log(Logger::Level::INFO) << "Maximum node number: " << simNodeIdx << std::endl;
			mLog.Log(Logger::Level::INFO) << "Number of nodes: " << mNodes.size() << std::endl;
		}

		/// Creates virtual nodes inside components.
		/// The MNA algorithm handles these nodes in the same way as network nodes.
		void createVirtualNodes() {
			// virtual nodes are placed after network nodes
			UInt virtualNode = mNumRealNodes - 1;	

			// Check if component requires virtual node and if so set one
			for (auto comp : mPowerComponents) {
				if (comp->hasVirtualNodes()) {
					for (UInt node = 0; node < comp->getVirtualNodesNum(); node++) {
						virtualNode++;
						mNodes.push_back(std::make_shared<Node<VarType>>(virtualNode));
						comp->setVirtualNodeAt(mNodes[virtualNode], node);
						mLog.Log(Logger::Level::INFO) << "Created virtual node" << node << " = " << virtualNode
							<< " for " << comp->getName() << std::endl;
					}
				}
			}

			// Calculate system size and create matrices and vectors
			mNumNodes = virtualNode + 1;
			mNumVirtualNodes = mNumNodes - mNumRealNodes;
		}

		// TODO: check if this works with AC sources
		void steadyStateInitialization() {
			Real time = 0;

			// Initialize right side vector and components
			for (auto comp : mPowerComponents) {
				comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep);
				comp->mnaApplyRightSideVectorStamp(mRightSideVector);
				comp->mnaApplySystemMatrixStamp(mSystemMatrices[0]);
				comp->mnaApplyInitialSystemMatrixStamp(mSystemMatrices[0]);
			}
			// Compute LU-factorization for system matrix
			mLuFactorizations.push_back(Eigen::PartialPivLU<Matrix>(mSystemMatrices[0]));

			Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
			Matrix diff;
			Real maxDiff, max;

			while (time < 10) {
				time = step(time);

				diff = prevLeftSideVector - mLeftSideVector;
				prevLeftSideVector = mLeftSideVector;
				maxDiff = diff.lpNorm<Eigen::Infinity>();
				max = mLeftSideVector.lpNorm<Eigen::Infinity>();

				if ((maxDiff / max) < 0.0001)
					break;
			}
			mLog.Log(Logger::Level::INFO) << "Initialization finished. Max difference: "
				<< maxDiff << " or " << maxDiff / max << "% at time " << time << std::endl;
			mSystemMatrices.pop_back();
			mLuFactorizations.pop_back();
			mSystemIndex = 0;

			createEmptySystemMatrix();
		}

		/// Create left and right side vector	
		void createEmptyVectors();

		/// Create system matrix
		void createEmptySystemMatrix();

		/// TODO remove and replace with function that handles breakers
		void addSystemTopology(SystemTopology system) {
			mSystem = system;

			// It is assumed that the system size does not change
			createEmptySystemMatrix();
			for (auto comp : system.mComponents)
				comp->mnaApplySystemMatrixStamp(mSystemMatrices[mSystemMatrices.size()]);

			mLuFactorizations.push_back(LUFactorized(mSystemMatrices[mSystemMatrices.size()]));
		}

		/// TODO This should be activated by switch/breaker components
		void switchSystemMatrix(UInt systemIndex) {
			if (systemIndex < mSystemMatrices.size())
				mSystemIndex = systemIndex;
		}

		/// Solve system matrices
		void solve()  {
			mLeftSideVector = mLuFactorizations[mSystemIndex].solve(mRightSideVector);
		}

	public:
		/// This constructor should not be called by users.
		MnaSolver(String name,
			Real timeStep,
			Domain domain = Domain::DP,
			Logger::Level logLevel = Logger::Level::INFO,
			Bool steadyStateInit = false, Int downSampleRate = 1) :			
			mLog("Logs/" + name + "_MNA.log", logLevel),
			mLeftVectorLog("Logs/" + name + "_LeftVector.csv", logLevel),
			mRightVectorLog("Logs/" + name + "_RightVector.csv", logLevel) {

			mTimeStep = timeStep;
			mDomain = domain;
			mLogLevel = logLevel;
			mDownSampleRate = downSampleRate;
			mSteadyStateInit = steadyStateInit;
		}

		/// Constructor to be used in simulation examples.
		MnaSolver(String name, SystemTopology system,
			Real timeStep,
			Domain domain = Domain::DP,
			Logger::Level logLevel = Logger::Level::INFO,
			Bool steadyStateInit = false,
			Int downSampleRate = 1)
			: MnaSolver(name, timeStep, domain,
			logLevel, steadyStateInit, downSampleRate) {
			initialize(system);

			// Logging
			for (auto comp : system.mComponents)
				mLog.Log(Logger::Level::INFO) << "Added " << comp->getType() << " '" << comp->getName() << "' to simulation." << std::endl;

			mLog.Log(Logger::Level::INFO) << "System matrix:" << std::endl;
			mLog.LogMatrix(Logger::Level::INFO, mSystemMatrices[0]);
			mLog.Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
			mLog.LogMatrix(Logger::Level::INFO, mLuFactorizations[0].matrixLU());
			mLog.Log(Logger::Level::INFO) << "Right side vector:" << std::endl;
			mLog.LogMatrix(Logger::Level::INFO, mRightSideVector);
		}

		///
		virtual ~MnaSolver() { };

		/// Solve system A * x = z for x and current time
		Real step(Real time) {
			mRightSideVector.setZero();

			// First, step signal components and then power components
			for (auto comp : mSignalComponents) {
				comp->step(time);
			}
			for (auto comp : mPowerComponents) {
				comp->mnaStep(mSystemMatrices[mSystemIndex], mRightSideVector, mLeftSideVector, time);
			}

			// Solve MNA system
			solve();

			// Some components need to update internal states
			for (auto comp : mPowerComponents) {
				comp->mnaPostStep(mRightSideVector, mLeftSideVector, time);
			}

			// TODO Try to avoid this step.
			for (UInt nodeIdx = 0; nodeIdx < mNumRealNodes; nodeIdx++) {
				mNodes[nodeIdx]->mnaUpdateVoltage(mLeftSideVector);
			}

			// Handle switching events
			if (mSwitchTimeIndex < mSwitchEvents.size()) {
				if (time >= mSwitchEvents[mSwitchTimeIndex].switchTime) {
					switchSystemMatrix(mSwitchEvents[mSwitchTimeIndex].systemIndex);
					++mSwitchTimeIndex;

					mLog.Log(Logger::Level::INFO) << "Switched to system " << mSwitchTimeIndex << " at " << time << std::endl;
					mLog.Log(Logger::Level::INFO) << "New matrix:" << std::endl << mSystemMatrices[mSystemIndex] << std::endl;
					mLog.Log(Logger::Level::INFO) << "New decomp:" << std::endl << mLuFactorizations[mSystemIndex].matrixLU() << std::endl;
				}
			}

			// Calculate new simulation time
			return time + mTimeStep;
		}

		/// Log left and right vector values for each simulation step
		void log(Real time) {
			mLeftVectorLog.LogNodeValues(time, getLeftSideVector());
			mRightVectorLog.LogNodeValues(time, getRightSideVector());
		}

		/// TODO This should be handled by a switch/breaker component
		void setSwitchTime(Real switchTime, Int systemIndex) {
			SwitchConfiguration newSwitchConf;
			newSwitchConf.switchTime = switchTime;
			newSwitchConf.systemIndex = systemIndex;
			mSwitchEvents.push_back(newSwitchConf);
		}

		// #### Getter ####
		Matrix& getLeftSideVector() { return mLeftSideVector; }
		Matrix& getRightSideVector() { return mRightSideVector; }
		Matrix& getSystemMatrix() { return mSystemMatrices[mSystemIndex]; }
	};

	
}
