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

#include "CPowerSystems/Source/Definitions.h"
#include "CPowerSystems/Source/Component.h"
#include "CPowerSystems/Source/Logger.h"
#include "CPowerSystems/Source/Interfaces/ExternalInterface.h"
#include "CPowerSystems/Source/Node.h"

namespace DPsim {
	/// Ground node
	const Int GND = -1;
	/// Holds switching time and which system should be activated.
	struct SwitchConfiguration {
		Real switchTime;
		UInt systemIndex;
	};
	/// Simulation class which uses Modified Nodal Analysis (MNA).
	class MnaSimulation {
	protected:
		/// Simulation name
		String mName;

		// General simulation settings
		/// Final time of the simulation
		Real mFinalTime;
		/// Time variable that is incremented at every step
		Real mTime = 0;
		/// system frequency
		Real mSystemFrequency;
		/// System angular frequency - omega
		Real mSystemOmega;
		/// Simulation time step
		Real mTimeStep;
		/// Simulation type, which can be dynamic phasor (DP) or EMT
		SimulationType mSimType;
		/// Ground node
		Node::Ptr mGnd;
		/// Number of nodes
		UInt mNumNodes = 0;
		/// Vector of ExternalInterfaces
		std::vector<ExternalInterface*> mExternalInterfaces;
		/// Flag to activate power flow based initialization.
		/// If this is false, all voltages are initialized with zero.
		Bool mPowerflowInitialization;
		/// System matrix A that is modified by matrix stamps
		UInt mSystemIndex = 0;
		/// Stores all lists of electrical nodes
		std::vector<Node::List> mNodes;
		/// Circuit list vector
		std::vector<Component::List> mComponents;
		/// System matrices list for swtiching events
		std::vector<Matrix> mSystemMatrices;
		/// LU decomposition of system matrix A
		std::vector<LUFactorized> mLuFactorizations;
		/// Vector of known quantities
		Matrix mRightSideVector;
		/// Vector of unknown quantities
		Matrix mLeftSideVector;		
		/// Numerical integration method for components which are not part of the network
		NumericalMethod mNumMethod;

		// Variables related to switching
		/// Index of the next switching event
		UInt mSwitchTimeIndex = 0;
		/// Vector of switch times
		std::vector<SwitchConfiguration> mSwitchEvents;

		// Attributes related to logging
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
		void initialize(Component::List comps);
		/// Solve system A * x = z for x and current time
		Int step(bool blocking = true);
		/// Advance the simulation clock by 1 time-step.
		void increaseByTimeStep() { mTime = mTime + mTimeStep; }
		///
		void addSystemTopology(Component::List newComps);
		///
		void switchSystemMatrix(Int systemMatrixIndex);
		///
		void createEmptyVectors();
		///
		void createEmptySystemMatrix();
		///
		void solve();
	public:
		/// Creates system matrix according to
		MnaSimulation(String name,
			Component::List comps, Real frequency, Real timeStep, Real finalTime, SimulationType simType = SimulationType::DP,
			Logger::Level logLevel = Logger::Level::INFO, Int downSampleRate = 1);
		/// Creates system matrix according to
		MnaSimulation(String name,
			std::list<String> cimFiles,
			Real frequency, Real timeStep, Real finalTime, SimulationType simType = SimulationType::DP,
			Logger::Level logLevel = Logger::Level::INFO, Int downSampleRate = 1);
		///
		virtual ~MnaSimulation() { };
		/// Run simulation until total time is elapsed.
		void run();
		/// Run simulation for \p duration seconds.
		void run(double duration);		
		///
		void addExternalInterface(ExternalInterface* eint) { mExternalInterfaces.push_back(eint); }
		///
		void setSwitchTime(Real switchTime, Int systemIndex);

		// #### Getter ####
		String getName() const { return mName; }
		Real getTime() { return mTime; }
		Real getFinalTime() { return mFinalTime; }
		Real getTimeStep() { return mTimeStep; }
		Matrix& getLeftSideVector() { return mLeftSideVector; }
		Matrix& getRightSideVector() { return mRightSideVector; }
		Matrix& getSystemMatrix() { return mSystemMatrices[mSystemIndex]; }
	};

}
