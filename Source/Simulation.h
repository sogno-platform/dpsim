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

#include "cps/Source/Definitions.h"
#include "cps/Source/Logger.h"
#include "cps/Source/SystemTopology.h"
#include "MNA_Solver.h"

namespace DPsim {

	class Simulation {

	protected:
		/// Simulation log level
		Logger::Level mLogLevel;
		/// Simulation logger
		Logger mLog;
		/// Simulation name
		String mName;
		///
		Solver::Type mSolverType;
		///
		std::shared_ptr<Solver> mSolver;
	public:
		enum class Type { DP, EMT };

		/// Creates system matrix according to
		Simulation(String name,
			Real timeStep, Real finalTime,
			Simulation::Type simType = Simulation::Type::DP,
			Solver::Type solverType = Solver::Type::MNA,
			Logger::Level logLevel = Logger::Level::INFO,
			Bool steadyStateInit = false);
		/// Creates system matrix according to
		Simulation(String name, SystemTopology system,
			Real timeStep, Real finalTime,
			Simulation::Type simType = Simulation::Type::DP,
			Solver::Type solverType = Solver::Type::MNA,
			Logger::Level logLevel = Logger::Level::INFO);
		/// Creates system matrix according to
		Simulation(String name, std::list<String> cimFiles, Real frequency,
			Real timeStep, Real finalTime,
			Simulation::Type simType = Simulation::Type::DP,
			Solver::Type solverType = Solver::Type::MNA,
			Logger::Level logLevel = Logger::Level::INFO);
		///
		virtual ~Simulation() { };
		/// Run simulation until total time is elapsed.
		void run();
		/// Run simulation for \p duration seconds.
		void run(double duration);
		///
		void setSwitchTime(Real switchTime, Int systemIndex);
		///
		void addExternalInterface(ExternalInterface*);
		///
		void addSystemTopology(SystemTopology system);
		///
		void setLogDownsamplingRate(Int divider) {}

		// #### Getter ####
		String getName() const { return mName; }
	};

}
