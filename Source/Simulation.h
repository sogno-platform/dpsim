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

#include "Definitions.h"
#include "Component.h"
#include "Logger.h"
#include "SystemModel.h"
#include "ExternalInterface.h"

namespace DPsim {

	/// Ground node
	const Int GND = -1;

	struct switchConfiguration {
		Real switchTime;
		UInt systemIndex;
	};

	class Simulation {

	protected:
		/// Simulation name
		String mName;
		/// Simulation log level
		Logger::Level mLogLevel;
		/// Simulation logger
		Logger mLog;
		/// Left side vector logger
		Logger mLeftVectorLog;
		/// Right side vector logger
		Logger mRightVectorLog;
		/// Final time of the simulation
		Real mFinalTime;
		/// Time variable that is incremented at every step
		Real mTime;
		/// Last simulation time step when log was updated
		Int mLastLogTimeStep;
		/// Down sampling rate
		Int mDownSampleRate;
		/// Index of the next switching
		UInt mCurrentSwitchTimeIndex;
		/// Vector of switch times
		std::vector<switchConfiguration> mSwitchEventVector;
		/// Structure that holds all system information.
		SystemModel mSystemModel;
		/// Stores a list of circuit elements that are used to generate the system matrix
		Component::List mComponents;
		/// Circuit list vector
		std::vector<Component::List> mComponentsVector;
		/// Vector of ExternalInterfaces
		std::vector<ExternalInterface*> mExternalInterfaces;

	public:
		/// Creates system matrix according to
		Simulation(String name, Component::List comps, Real om, Real dt, Real tf, Logger::Level logLevel = Logger::Level::INFO, SimulationType simType = SimulationType::DP, Int downSampleRate = 1);
		virtual ~Simulation() { };

		/// TODO: check that every system matrix has the same dimensions
		void initialize(Component::List comps);
		/// Solve system A * x = z for x and current time
		Int step(bool blocking = true);
		/// Run simulation until total time is elapsed.
		void run();
		/// Run simulation for \p duration seconds.
		void run(double duration);
		/// Advance the simulation clock by 1 time-step.
		void increaseByTimeStep();

		void switchSystemMatrix(Int systemMatrixIndex);
		void setSwitchTime(Real switchTime, Int systemIndex);

		void addExternalInterface(ExternalInterface*);

		void setNumericalMethod(NumericalMethod numMethod);

		// Getter
		String getName() const { return mName; }
		Real getTime() { return mTime; }
		Real getFinalTime() { return mFinalTime; }
		Real getTimeStep() { return mSystemModel.getTimeStep(); }
		Matrix & getLeftSideVector() { return mSystemModel.getLeftSideVector(); }
		Matrix & getRightSideVector() { return mSystemModel.getRightSideVector(); }
		Matrix & getSystemMatrix() { return mSystemModel.getCurrentSystemMatrix(); }

		void addSystemTopology(Component::List newComps);
	};

}
