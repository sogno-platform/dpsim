/**
 * @file
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
#include <cstdint>

#include <dpsim/Config.h>
#include <dpsim/DataLogger.h>
#include <dpsim/Solver.h>
#include <dpsim/Event.h>
#include <dpsim/Scheduler.h>
#include <cps/Definitions.h>
#include <cps/PowerComponent.h>
#include <cps/Logger.h>
#include <cps/SystemTopology.h>
#include <cps/Node.h>

#ifdef WITH_SHMEM
  #include <dpsim/Interface.h>
#endif

namespace DPsim {
	/// \brief The Simulation holds a SystemTopology and a Solver.
	///
	/// Every time step, the Simulation calls the step function of the Solver.
	class Simulation :
		public CPS::AttributeList {
	public:
		typedef std::shared_ptr<Simulation> Ptr;

	protected:
		/// Simulation logger
		CPS::Logger mLog;
		/// Simulation name
		String mName;
		/// Final time of the simulation
		Real mFinalTime;
		///
		CPS::Domain mDomain;
		/// Time variable that is incremented at every step
		Real mTime = 0;
		/// Simulation timestep
		Real mTimeStep;
		/// Number of step which have been executed for this simulation.
		Int mTimeStepCount = 0;
		/// Simulation log level
		CPS::Logger::Level mLogLevel;
		///
		Solver::List mSolvers;
		/// The simulation event queue
		EventQueue mEvents;
		/// Scheduler used for task scheduling
		std::shared_ptr<Scheduler> mScheduler;
		/// List of all tasks to be scheduled
		CPS::Task::List mTasks;
		/// Task dependencies as incoming / outgoing edges
		Scheduler::Edges mTaskInEdges, mTaskOutEdges;
		/// (Real) time needed for the timesteps
		std::vector<Real> mStepTimes;

#ifdef WITH_SHMEM
		struct InterfaceMapping {
			/// A pointer to the external interface
			Interface *interface;
			/// Is this interface used for synchronization of the simulation start?
			bool syncStart;
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
		DataLogger::List mLoggers;

		/// Creates system matrix according to
		Simulation(String name,
			Real timeStep, Real finalTime,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO);

		template <typename VarType>
		void createSolvers(const CPS::SystemTopology& system, Solver::Type solverType, Bool steadyStateInit, Bool splitSubnets, const CPS::Component::List& tearComponents);

		template <typename VarType>
		static int checkTopologySubnets(const CPS::SystemTopology& system, std::unordered_map<typename CPS::Node<VarType>::Ptr, int>& subnet);

		void prepSchedule();
	public:
		/// Creates system matrix according to a given System topology
		Simulation(String name, CPS::SystemTopology system,
			Real timeStep, Real finalTime,
			CPS::Domain domain = CPS::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO,
			Bool steadyStateInit = false,
			Bool splitSubnets = true,
			CPS::Component::List tearComponents = CPS::Component::List());
		/// Desctructor
		virtual ~Simulation();

		/// Run simulation until total time is elapsed.
		void run();
		/// Solve system A * x = z for x and current time
		virtual Real step();
		/// Synchronize simulation with remotes by exchanging intial state over interfaces
		void sync();
		/// Create the schedule for the independent tasks
		void schedule();
#ifdef WITH_GRAPHVIZ
		void renderDependencyGraph(std::ostream& os);
#endif

		template <typename VarType>
		static void splitSubnets(const CPS::SystemTopology& system, std::vector<CPS::SystemTopology>& splitSystems);

		/// Schedule an event in the simulation
		void addEvent(Event::Ptr e) {
			mEvents.addEvent(e);
		}
#ifdef WITH_SHMEM
		void addInterface(Interface *eint, Bool syncStart = true) {
			mInterfaces.push_back({eint, syncStart});
		}

		std::vector<InterfaceMapping> & interfaces() { return mInterfaces; }
#endif
		void addLogger(DataLogger::Ptr logger) {
			mLoggers.push_back(logger);
		}

		void setScheduler(std::shared_ptr<Scheduler> scheduler) {
			mScheduler = scheduler;
		}

		// #### Getter ####
		String name() const { return mName; }
		Real time() const { return mTime; }
		Real finalTime() const { return mFinalTime; }
		Int timeStepCount() const { return mTimeStepCount; }
		Real timeStep() const { return mTimeStep; }
		DataLogger::List& loggers() { return mLoggers; }
		std::shared_ptr<Scheduler> scheduler() { return mScheduler; }
		std::vector<Real>& stepTimes() { return mStepTimes; }
	};

}
