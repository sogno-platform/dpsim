/**
 * @file
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

#include <dpsim/Config.h>
#include <dpsim/DataLogger.h>
#include <dpsim/Solver.h>
#include <dpsim/Event.h>
#include <cps/Definitions.h>
#include <cps/PowerComponent.h>
#include <cps/Logger.h>
#include <cps/SystemTopology.h>
#include <cps/Node.h>

#ifdef WITH_SHMEM
  #include <cps/Interface.h>
#endif

namespace DPsim {

	class Simulation : public CPS::AttributeList {
	public:
		typedef std::shared_ptr<Simulation> Ptr;

	protected:
		/// Simulation logger
		CPS::Logger mLog;
		/// Simulation name
		String mName;
		/// Final time of the simulation
		Real mFinalTime;
		/// Time variable that is incremented at every step
		Real mTime = 0;
		/// Simulation timestep
		Real mTimeStep;
		/// Number of step which have been executed for this simulation.
		Int mTimeStepCount = 0;
		/// Simulation log level
		CPS::Logger::Level mLogLevel;
		///
		Solver::Type mSolverType;
		///
		std::shared_ptr<Solver> mSolver;
		/// The simulation event queue
		EventQueue mEvents;

#ifdef WITH_SHMEM
		struct InterfaceMapping {
			/// A pointer to the external interface
			CPS::Interface *interface;
			/// Is this interface used for synchorinzation?
			bool sync;
			/// Is this interface used for synchronization of the simulation start?
			bool syncStart;
			/// Downsampling
			UInt downsampling;
		};

		/// Vector of Interfaces
		std::vector<InterfaceMapping> mInterfaces;
#endif /* WITH_SHMEM */

		struct LoggerMapping {
			/// Simulation data logger
			DataLogger::Ptr logger;
			/// Downsampling
			UInt downsampling;
		};

		/// The data loggers
		std::vector<LoggerMapping> mLoggers;

	public:
		/// Creates system matrix according to
		Simulation(String name,
			Real timeStep, Real finalTime,
			CPS::Domain domain = CPS::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO);
		/// Creates system matrix according to System topology
		Simulation(String name, CPS::SystemTopology system,
			Real timeStep, Real finalTime,
			CPS::Domain domain = CPS::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO,
			Bool steadyStateInit = false);
		///
		virtual ~Simulation();

		/// Run simulation until total time is elapsed.
		void run();
		/// Solve system A * x = z for x and current time
		Real step();
		/// Synchronize simulation with remotes by exchanging intial state over interfaces
		void sync();

		/// Schedule an event in the simulation
		void addEvent(Event::Ptr e) {
			mEvents.addEvent(e);
		}
#ifdef WITH_SHMEM
		///
		void addInterface(CPS::Interface *eint, Bool sync, Bool syncStart, UInt downsampling = 1) {
			mInterfaces.push_back({eint, sync, syncStart, downsampling});
		}

		void addInterface(CPS::Interface *eint, Bool sync = true) {
			addInterface(eint, sync, sync);
		}
#endif
		void addLogger(DataLogger::Ptr logger, UInt downsampling = 1) {
			mLoggers.push_back({logger, downsampling});
		}

		// #### Getter ####
		String name() const { return mName; }
		Real time() const { return mTime; }
		Real finalTime() const { return mFinalTime; }
		Int timeStepCount() const { return mTimeStepCount; }
		Real timeStep() const { return mTimeStep; }
		std::vector<LoggerMapping> & loggers() { return mLoggers; }
		std::vector<InterfaceMapping> & interfaces() { return mInterfaces; }
	};

}
