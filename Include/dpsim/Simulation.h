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

#include <vector>

#include <dpsim/Config.h>
#include <dpsim/DataLogger.h>
#include <dpsim/Solver.h>
#include <dpsim/Scheduler.h>
#include <dpsim/Event.h>
#include <cps/Definitions.h>
#include <cps/Logger.h>
#include <cps/SystemTopology.h>
#include <cps/Node.h>

#ifdef WITH_GRAPHVIZ
  #include <cps/Graph.h>
#endif

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
		/// Simulation name
		String mName;
		/// Final time of the simulation
		Real mFinalTime = 0.1;
		/// Time variable that is incremented at every step
		Real mTime = 0;
		/// Simulation timestep
		Real mTimeStep = 0.001;
		/// Number of step which have been executed for this simulation.
		Int mTimeStepCount = 0;
		/// The simulation event queue
		EventQueue mEvents;
		/// System list
		CPS::SystemTopology mSystem;

		// #### Logging ####
		/// Simulation log level
		CPS::Logger::Level mLogLevel;
		/// (Real) time needed for the timesteps
		std::vector<Real> mStepTimes;

		// #### Solver Settings ####
		///
		CPS::Domain mDomain = CPS::Domain::DP;
		///
		Solver::Type mSolverType = Solver::Type::MNA;
		///
		Solver::List mSolvers;
		/// Determines if steady-state initialization
		/// should be executed prior to the simulation.
		/// By default the initialization is disabled.
		Bool mSteadyStateInit = false;
		/// Determines if the network should be split
		/// into subnetworks at decoupling lines.
		/// If the system is split, each subsystem is
		/// solved by a dedicated MNA solver.
		Bool mSplitSubnets = true;
		/// If tearing components exist, the Diakoptics
		/// solver is selected automatically.
		CPS::Component::List mTearComponents = CPS::Component::List();
		/// Determines if the system matrix is split into
		/// several smaller matrices, one for each frequency.
		/// This can only be done if the network is composed
		/// of linear components that do no create cross
		/// frequency coupling.
		Bool mHarmParallel = false;
		///
		Bool mInitialized = false;

		// #### Task dependencies und scheduling ####
		/// Scheduler used for task scheduling
		std::shared_ptr<Scheduler> mScheduler;
		/// List of all tasks to be scheduled
		CPS::Task::List mTasks;
		/// Task dependencies as incoming / outgoing edges
		Scheduler::Edges mTaskInEdges, mTaskOutEdges;

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
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		template <typename VarType>
		void createSolvers(CPS::SystemTopology& system, CPS::Component::List& tearComponents);

		void prepSchedule();
	public:
		/// Simulation logger
		CPS::Logger::Log mSLog;
		/// Simulation logger (console)
		CPS::Logger::Log mCLog;

		/// Creates simulation with name and log level
		Simulation(String name, CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		/// Creates system matrix according to a given System topology
		Simulation(String name, CPS::SystemTopology system,
			Real timeStep, Real finalTime,
			CPS::Domain domain = CPS::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info,
			Bool steadyStateInit = false,
			Bool splitSubnets = true,
			CPS::Component::List tearComponents = CPS::Component::List());

		/// Desctructor
		virtual ~Simulation() { }

		// #### Simulation Settings ####
		///
		void setSystem(CPS::SystemTopology system) { mSystem = system; }
		///
		void setTimeStep(Real timeStep) { mTimeStep = timeStep; }
		///
		void setFinalTime(Real finalTime) { mFinalTime = finalTime; }
		///
		void setDomain(CPS::Domain domain = CPS::Domain::DP) { mDomain = domain; }
		///
		void setSolverType(Solver::Type solverType = Solver::Type::MNA) { mSolverType = solverType; }
		///
		void doSteadyStateInit(Bool steadyStateInit = false) { mSteadyStateInit = steadyStateInit; }
		///
		void doSplitSubnets(Bool splitSubnets = true) { mSplitSubnets = splitSubnets; }
		///
		void setTearingComponents(CPS::Component::List tearComponents = CPS::Component::List()) {
			mTearComponents = tearComponents;
		}
		/// Set the scheduling method
		void setScheduler(std::shared_ptr<Scheduler> scheduler) {
			mScheduler = scheduler;
		}
		///
		void doHarmonicParallelization(Bool parallel) { mHarmParallel = parallel; }

		// #### Simulation Control ####
		/// Create solver instances etc.
		void initialize();
		/// Run simulation until total time is elapsed.
		void run();
		/// Solve system A * x = z for x and current time
		virtual Real step();
		/// Synchronize simulation with remotes by exchanging intial state over interfaces
		void sync();
		/// Create the schedule for the independent tasks
		void schedule();
		/// Reset internal state of simulation
		void reset();

		/// Schedule an event in the simulation
		void addEvent(Event::Ptr e) {
			mEvents.addEvent(e);
		}
		/// Add a new data logger
		void addLogger(DataLogger::Ptr logger) {
			mLoggers.push_back(logger);
		}
		/// Write step time measurements to log file
		void logStepTimes(String logName);

#ifdef WITH_SHMEM
		///
		void addInterface(Interface *eint, Bool syncStart = true) {
			mInterfaces.push_back({eint, syncStart});
		}
		/// Return list of interfaces
		std::vector<InterfaceMapping> & interfaces() { return mInterfaces; }
#endif
#ifdef WITH_GRAPHVIZ
		///
		CPS::Graph::Graph dependencyGraph();
#endif

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
