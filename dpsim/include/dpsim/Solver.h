/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <list>
#include <vector>

#include <dpsim-models/Logger.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim-models/Task.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/DirectLinearSolverConfiguration.h>

namespace DPsim {
/// Holds switching time and which system should be activated.
struct SwitchConfiguration {
  Real switchTime;
  UInt systemIndex;
};

/// Base class for more specific solvers such as MNA, ODE or IDA.
class Solver {
public:
  typedef std::shared_ptr<Solver> Ptr;
  typedef std::vector<Ptr> List;

  enum Behaviour { Initialization, Simulation };

  /// System-matrix recomputation mode for MNA solvers.
  enum class SystemMatrixRecomputationMode {
    /// Select the mode automatically based on the system topology.
    Auto,
    /// Always enable system-matrix recomputation.
    Enabled,
    /// Always disable system-matrix recomputation.
    Disabled
  };

protected:
  /// Name for logging
  String mName;
  /// Logging level
  CPS::Logger::Level mLogLevel;
  /// Collect step time for logging
  Bool mLogSolveTimes = true;
  /// Logger
  CPS::Logger::Log mSLog;
  /// Time step for fixed step solvers
  Real mTimeStep;
  /// Activates parallelized computation of frequencies
  Bool mFrequencyParallel = false;

  // #### Initialization ####
  /// steady state initialization time limit
  Real mSteadStIniTimeLimit = 10;
  /// steady state initialization accuracy limit
  Real mSteadStIniAccLimit = 0.0001;
  /// Activates steady state initialization
  Bool mSteadyStateInit = false;
  /// Determines if solver is in initialization phase, which requires different behavior
  Bool mIsInInitialization = false;
  /// Activates powerflow initialization
  /// If this is false, all voltages are initialized with zero
  Bool mInitFromNodesAndTerminals = true;
  /// Requested system-matrix recomputation mode.
  SystemMatrixRecomputationMode mSystemMatrixRecomputationMode =
      SystemMatrixRecomputationMode::Auto;
  /// Effective system-matrix recomputation setting used by the solver.
  Bool mSystemMatrixRecomputationEnabled = false;

  /// Solver behaviour initialization or simulation
  Behaviour mBehaviour = Solver::Behaviour::Simulation;

public:
  Solver(String name, CPS::Logger::Level logLevel)
      : mName(name), mLogLevel(logLevel),
        mSLog(CPS::Logger::get(name + "_Solver", logLevel,
                               CPS::Logger::Level::warn)) {}

  virtual ~Solver() {}

  // #### Solver settings ####
  /// Solver types:
  /// Modified Nodal Analysis, Differential Algebraic, Newton Raphson
  enum class Type { MNA, DAE, NRP };
  ///
  void setTimeStep(Real timeStep) { mTimeStep = timeStep; }
  ///
  void doFrequencyParallelization(Bool freqParallel) {
    mFrequencyParallel = freqParallel;
  }
  ///
  virtual void setSystem(const CPS::SystemTopology &system) {}
  /// Set the system-matrix recomputation mode.
  void setSystemMatrixRecomputationMode(SystemMatrixRecomputationMode mode) {
    mSystemMatrixRecomputationMode = mode;
  }
  /// DEPRECATED: Enable or disable system-matrix recomputation explicitly.
  /// This compatibility method maps true to Enabled and false to Disabled.
  void doSystemMatrixRecomputation(Bool value) {
    setSystemMatrixRecomputationMode(
        value ? SystemMatrixRecomputationMode::Enabled
              : SystemMatrixRecomputationMode::Disabled);
  }

  void setLogSolveTimes(Bool value) { mLogSolveTimes = value; }

  // #### Initialization ####
  ///
  virtual void initialize() {}
  /// activate steady state initialization
  void doSteadyStateInit(Bool f) { mSteadyStateInit = f; }
  /// set steady state initialization time limit
  void setSteadStIniTimeLimit(Real v) { mSteadStIniTimeLimit = v; }
  /// set steady state initialization accuracy limit
  void setSteadStIniAccLimit(Real v) { mSteadStIniAccLimit = v; }
  /// set solver and component to initialization or simulation behaviour
  virtual void setSolverAndComponentBehaviour(Solver::Behaviour behaviour) {}
  /// activate powerflow initialization
  void doInitFromNodesAndTerminals(Bool f) { mInitFromNodesAndTerminals = f; }
  /// set direct linear solver configuration (only available in MNA for now)
  virtual void
  setDirectLinearSolverConfiguration(DirectLinearSolverConfiguration &) {
    // not every derived class has a linear solver configuration option
  }
  /// log LU decomposition times, if applicable
  virtual void logLUTimes() {
    // no default implementation for all types of solvers
  }

  // #### Simulation ####
  /// Get tasks for scheduler
  virtual CPS::Task::List getTasks() = 0;
  /// Log results
  virtual void log(Real time, Int timeStepCount){};

  /// ### SynGen Interface ###
  int mMaxIterations = 10;
  void setMaxNumberOfIterations(int maxIterations) {
    mMaxIterations = maxIterations;
  }
};
} // namespace DPsim
