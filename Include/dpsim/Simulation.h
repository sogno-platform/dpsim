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
#include <cstdint>

#include "Config.h"
#include "MNA_Solver.h"

#include "cps/Definitions.h"
#include "cps/Component.h"
#include "cps/Logger.h"
#include "cps/SystemTopology.h"
#include "cps/Node.h"

#ifdef WITH_SHMEM
  #include "cps/Interface.h"
#endif

using namespace CPS;

namespace DPsim {

	class Simulation : public AttributeList {

	public:
		enum class Event : std::uint32_t {
			Started = 1,
			Stopped = 2,
			Finished = 3,
			Overrun = 4,
			Paused = 5,
			Resumed = 6
		};

	protected:
		/// Simulation logger
		Logger mLog;
		/// Simulation name
		String mName;
		/// Final time of the simulation
		Real mFinalTime;
		/// Time variable that is incremented at every step
		Real mTime = 0;
		/// Number of step which have been executed for this simulation.
		Int mTimeStepCount = 0;
		/// Simulation log level
		Logger::Level mLogLevel;
		///
		Solver::Type mSolverType;
		///
		std::shared_ptr<Solver> mSolver;
		/// Pipe for asynchronous inter-process communication (IPC) to the Python world
		int mPipe[2];

	public:
		/// Creates system matrix according to
		Simulation(String name,
			Real timeStep, Real finalTime,
			Solver::Domain domain = Solver::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			Logger::Level logLevel = Logger::Level::INFO);
		/// Creates system matrix according to System topology
		Simulation(String name, SystemTopology system,
			Real timeStep, Real finalTime,
			Solver::Domain domain = Solver::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			Logger::Level logLevel = Logger::Level::INFO,
			Bool steadyStateInit = false);
		///
		virtual ~Simulation();

		/// Run simulation until total time is elapsed.
		void run(bool blocking = true);
		/// Run simulation for \p duration seconds.
		void run(double duration, bool blocking = true);
		/// Solve system A * x = z for x and current time
		Real step(bool blocking = true);

		///
		void setSwitchTime(Real switchTime, Int systemIndex);
#ifdef WITH_SHMEM
		///
		void addInterface(Interface*);
#endif
		///
		void addSystemTopology(SystemTopology system);
		///
		void setLogDownsamplingRate(Int divider) {}

		// #### Getter ####
		String getName() const { return mName; }
		Real getTime() const { return mTime; }
		Real getFinalTime() const { return mFinalTime; }
		Int getTimeStepCount() const { return mTimeStepCount; }
		int getEventFD(Int flags = -1, Int coalesce = 1);

		/// Sends a notification to other processes / Python
		void sendNotification(enum Event evt);
	};

}
