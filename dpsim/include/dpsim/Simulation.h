// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "dpsim/MNASolverFactory.h"
#include <vector>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim/Config.h>
#include <dpsim/DataLogger.h>
#include <dpsim/Event.h>
#include <dpsim/Interface.h>
#include <dpsim/Scheduler.h>
#include <dpsim/Solver.h>
#include <nlohmann/json.hpp>

#ifdef WITH_GRAPHVIZ
#include <dpsim-models/Graph.h>
#endif

using json = nlohmann::json;

namespace DPsim {
/// Forward declaration of CommandLineArgs from Utils
class CommandLineArgs;

/// \brief The Simulation holds a SystemTopology and a Solver.
///
/// Every time step, the Simulation calls the step function of the Solver.
class Simulation : public CPS::AttributeList {
public:
  typedef std::shared_ptr<Simulation> Ptr;

  /// Simulation name
  const CPS::Attribute<String>::Ptr mName;
  /// If there we use a solver plugin, this specifies its name (excluding .so)
  String mSolverPluginName;
  /// Final time of the simulation
  const CPS::Attribute<Real>::Ptr mFinalTime;
  /// Simulation timestep
  const CPS::Attribute<Real>::Ptr mTimeStep;

  /// Determines if the network should be split
  /// into subnetworks at decoupling lines.
  /// If the system is split, each subsystem is
  /// solved by a dedicated MNA solver.
  const CPS::Attribute<Bool>::Ptr mSplitSubnets;

  /// Determines if steady-state initialization
  /// should be executed prior to the simulation.
  /// By default the initialization is disabled.
  const CPS::Attribute<Bool>::Ptr mSteadyStateInit;

protected:
  /// Time variable that is incremented at every step
  Real mTime = 0;
  /// Number of step which have been executed for this simulation.
  Int mTimeStepCount = 0;
  /// The simulation event queue
  EventQueue mEvents;
  /// System list
  CPS::SystemTopology mSystem;

  /// Start time point to measure calculation time
  std::chrono::time_point<std::chrono::steady_clock> mSimulationStartTimePoint;
  /// End time point to measure calculation time
  std::chrono::time_point<std::chrono::steady_clock> mSimulationEndTimePoint;
  /// Measured calculation time for simulation
  std::chrono::duration<double> mSimulationCalculationTime;

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
  Solver::Behaviour mSolverBehaviour = Solver::Behaviour::Simulation;
  ///
  Solver::List mSolvers;
  ///
  DirectLinearSolverImpl mDirectImpl = DirectLinearSolverImpl::Undef;
  ///
  DirectLinearSolverConfiguration mDirectLinearSolverConfiguration;
  ///
  Bool mInitFromNodesAndTerminals = true;
  /// Enable recomputation of system matrix during simulation
  Bool mSystemMatrixRecomputation = false;

  /// If tearing components exist, the Diakoptics
  /// solver is selected automatically.
  CPS::IdentifiedObject::List mTearComponents = CPS::IdentifiedObject::List();
  /// Determines if the system matrix is split into
  /// several smaller matrices, one for each frequency.
  /// This can only be done if the network is composed
  /// of linear components that do no create cross
  /// frequency coupling.
  Bool mFreqParallel = false;
  ///
  Bool mInitialized = false;

  // #### Initialization ####
  /// steady state initialization time limit
  Real mSteadStIniTimeLimit = 10;
  /// steady state initialization accuracy limit
  Real mSteadStIniAccLimit = 0.0001;

  // #### Task dependencies und scheduling ####
  /// Scheduler used for task scheduling
  std::shared_ptr<Scheduler> mScheduler;
  /// List of all tasks to be scheduled
  CPS::Task::List mTasks;
  /// Task dependencies as incoming / outgoing edges
  Scheduler::Edges mTaskInEdges, mTaskOutEdges;

  /// Vector of Interfaces
  std::vector<Interface::Ptr> mInterfaces;

  struct LoggerMapping {
    /// Simulation data logger
    DataLogger::Ptr logger;
    /// Downsampling
    UInt downsampling;
  };

  /// The data loggers
  DataLogger::List mLoggers;

  /// Helper function for constructors
  void create();
  /// Create solvers depending on simulation settings
  template <typename VarType> void createSolvers();
  /// Subroutine for MNA only because there are many MNA options
  template <typename VarType> void createMNASolver();

  template <typename VarType> void createIterativeMNASolver();
  // Prepare schedule for simulation
  void prepSchedule();

  /// ### SynGen Interface ###
  int mMaxIterations = 10;

public:
  /// Simulation logger
  CPS::Logger::Log mLog;

  /// Creates simulation with name and CommandLineArgs
  Simulation(String name, CommandLineArgs &args);

  /// Creates simulation with name and log level
  Simulation(String name,
             CPS::Logger::Level logLevel = CPS::Logger::Level::info);

  /// Desctructor
  virtual ~Simulation() {}

  // #### Simulation Settings ####
  ///
  void setSystem(const CPS::SystemTopology &system) { mSystem = system; }
  ///
  void setTimeStep(Real timeStep) { **mTimeStep = timeStep; }
  ///
  void setFinalTime(Real finalTime) { **mFinalTime = finalTime; }
  ///
  void setDomain(CPS::Domain domain = CPS::Domain::DP) { mDomain = domain; }
  ///
  void setSolverType(Solver::Type solverType = Solver::Type::MNA) {
    mSolverType = solverType;
  }
  /// set solver and component to initialization or simulation behaviour
  void setSolverAndComponentBehaviour(Solver::Behaviour behaviour) {
    mSolverBehaviour = behaviour;
  }
  ///
  void setDirectLinearSolverImplementation(DirectLinearSolverImpl directImpl) {
    mDirectImpl = directImpl;
  }
  ///
  void setDirectLinearSolverConfiguration(
      const DirectLinearSolverConfiguration &configuration) {
    mDirectLinearSolverConfiguration = configuration;
  }
  ///
  void setMaxNumberOfIterations(int maxIterations) {
    mMaxIterations = maxIterations;
  }
  ///
  void doInitFromNodesAndTerminals(Bool f = true) {
    mInitFromNodesAndTerminals = f;
  }
  ///
  void doSplitSubnets(Bool splitSubnets = true) {
    **mSplitSubnets = splitSubnets;
  }
  ///
  void setTearingComponents(CPS::IdentifiedObject::List tearComponents =
                                CPS::IdentifiedObject::List()) {
    mTearComponents = tearComponents;
  }
  /// Set the scheduling method
  void setScheduler(const std::shared_ptr<Scheduler> &scheduler) {
    mScheduler = scheduler;
  }
  /// Compute phasors of different frequencies in parallel
  void doFrequencyParallelization(Bool value) { mFreqParallel = value; }
  ///
  void doSystemMatrixRecomputation(Bool value) {
    mSystemMatrixRecomputation = value;
  }

  // #### Initialization ####
  /// activate steady state initialization
  void doSteadyStateInit(Bool f) { **mSteadyStateInit = f; }
  /// set steady state initialization time limit
  void setSteadStIniTimeLimit(Real v) { mSteadStIniTimeLimit = v; }
  /// set steady state initialization accuracy limit
  void setSteadStIniAccLimit(Real v) { mSteadStIniAccLimit = v; }

  // #### Simulation Control ####
  /// Create solver instances etc.
  void initialize();
  /// Start simulation without advancing in time
  void start();
  /// Stop simulation including scheduler and interfaces
  void stop();
  /// Run until next time step
  Real next();
  /// Run simulation until total time is elapsed.
  void run();
  /// Solve system A * x = z for x and current time
  virtual Real step();
  /// Synchronize simulation with remotes by exchanging intial state over interfaces
  void sync() const;
  /// Create the schedule for the independent tasks
  void schedule();

  /// Schedule an event in the simulation
  void addEvent(Event::Ptr e) { mEvents.addEvent(e); }
  /// Add a new data logger
  void addLogger(DataLogger::Ptr logger) { mLoggers.push_back(logger); }
  /// Write step time measurements to log file
  void logStepTimes(String logName);
  /// Check for overruns
  void checkForOverruns(String logName);

  /// Write LU decomposition times measurements to log file
  void logLUTimes();

  ///
  void addInterface(Interface::Ptr eint) {
    eint->setLogger(mLog);
    mInterfaces.push_back(eint);
  }

#ifdef WITH_GRAPHVIZ
  ///
  CPS::Graph::Graph dependencyGraph();
#endif

  // #### Getter ####
  String name() const { return **mName; }
  Real time() const { return mTime; }
  Real finalTime() const { return **mFinalTime; }
  Int timeStepCount() const { return mTimeStepCount; }
  Real timeStep() const { return **mTimeStep; }
  DataLogger::List &loggers() { return mLoggers; }
  std::shared_ptr<Scheduler> scheduler() { return mScheduler; }
  std::vector<Real> &stepTimes() { return mStepTimes; }

  // #### Set component attributes during simulation ####
  /// CHECK: Can these be deleted? getIdObjAttribute + "**attr =" should suffice
  // void setIdObjAttr(const String &comp, const String &attr, Real value);
  // void setIdObjAttr(const String &comp, const String &attr, Complex value);

  // #### Get component attributes during simulation ####
  CPS::AttributeBase::Ptr getIdObjAttribute(const String &comp,
                                            const String &attr);

  void logIdObjAttribute(const String &comp, const String &attr);
  /// CHECK: Can we store the attribute name / UID intrinsically inside the attribute?
  void logAttribute(String name, CPS::AttributeBase::Ptr attr);
};
} // namespace DPsim
