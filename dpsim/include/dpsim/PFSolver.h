/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cmath>
#include <iterator>

#include "dpsim-models/Components.h"
#include "dpsim-models/SystemTopology.h"
#include <dpsim/Scheduler.h>
#include <dpsim/Solver.h>

namespace DPsim {
/// Solver class using the nonlinear powerflow (PF) formulation.
class PFSolver : public Solver {
protected:
  /// Number of PQ nodes
  UInt mNumPQBuses = 0;
  /// Number of PV nodes
  UInt mNumPVBuses = 0;
  /// Number of PV nodes
  UInt mNumVDBuses = 0;
  /// Number of unknowns, defining system dimension
  UInt mNumUnknowns = 0;
  /// Vector of nodes characterized as PQ buses
  CPS::TopologicalNode::List mPQBuses;
  /// Vector of nodes characterized as PV buses
  CPS::TopologicalNode::List mPVBuses;
  /// Vector of nodes characterized as VD buses
  CPS::TopologicalNode::List mVDBuses;
  /// Original PQ/PV classification (snapshot before Q-limit switching)
  CPS::TopologicalNode::List mPQBusesOrig;
  CPS::TopologicalNode::List mPVBusesOrig;
  /// Vector with indices of PQ buses
  std::vector<CPS::UInt> mPQBusIndices;
  /// Vector with indices of PV buses
  std::vector<CPS::UInt> mPVBusIndices;
  /// Vector with indices of VD buses
  std::vector<CPS::UInt> mVDBusIndices;
  /// Vector with indices of both PQ and PV buses
  std::vector<CPS::UInt> mPQPVBusIndices;

  /// Admittance matrix
  CPS::SparseMatrixCompRow mY;

  /// Jacobian matrix
  CPS::Matrix mJ;
  /// Solution vector
  CPS::Vector mX;
  /// Vector of mismatch values
  CPS::Vector mF;

  /// System list
  CPS::SystemTopology mSystem;
  /// Vector of transformer components
  std::vector<std::shared_ptr<CPS::SP::Ph1::Transformer>> mTransformers;
  /// Vector of solid state transformer components
  std::vector<std::shared_ptr<CPS::SP::Ph1::SolidStateTransformer>>
      mSolidStateTransformers;
  /// Vector of synchronous generator components
  std::vector<std::shared_ptr<CPS::SP::Ph1::SynchronGenerator>>
      mSynchronGenerators;
  /// Vector of load components
  std::vector<std::shared_ptr<CPS::SP::Ph1::Load>> mLoads;
  /// Vector of line components
  std::vector<std::shared_ptr<CPS::SP::Ph1::PiLine>> mLines;
  /// Vector of shunt components
  std::vector<std::shared_ptr<CPS::SP::Ph1::Shunt>> mShunts;
  /// Vector of external grid components
  std::vector<std::shared_ptr<CPS::SP::Ph1::NetworkInjection>> mExternalGrids;
  /// Vector of average voltage source inverters
  std::vector<std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ>>
      mAverageVoltageSourceInverters;
  /// Map providing determined base voltages for each node
  std::map<CPS::TopologicalNode::Ptr, CPS::Real> mBaseVoltageAtNode;

  /// Solver tolerance
  Real mTolerance = 1e-8;
  /// Maximum number of iterations
  CPS::UInt mMaxIterations = 20;
  /// Actual number of iterations
  CPS::UInt mIterations;
  /// Enforce generator reactive-power limits via PV<->PQ outer-loop switching
  CPS::Bool mEnforceReactiveLimits = false;
  /// Maximum number of Q-limit outer iterations
  CPS::UInt mMaxOuterIterations = 10;
  /// Maximum number of PV<->PQ switches per bus before it is frozen (anti-oscillation)
  CPS::UInt mMaxQLimitSwitchesPerBus = 2;
  /// Base power of per-unit system
  CPS::Real mBaseApparentPower;
  /// Convergence flag
  CPS::Bool isConverged = false;
  /// Flag whether solution vectors are initialized
  CPS::Bool solutionInitialized = false;
  /// Flag whether complex solution vectors are initialized
  CPS::Bool solutionComplexInitialized = false;

  /// Use last converged solution as initial guess
  CPS::Bool mKeepLastSolution = false;

  /// Generate initial solution for current time step
  virtual void generateInitialSolution(Real time,
                                       bool keep_last_solution = false) = 0;
  /// Calculate mismatch
  virtual void calculateMismatch() = 0;
  /// Calculate the Jacobian
  virtual void calculateJacobian() = 0;
  /// Update solution in each iteration
  virtual void updateSolution() = 0;
  /// Set final solution
  virtual void setSolution() = 0;

  /// Initialization of the solver
  void initialize() override;
  /// Initialization of individual components
  void initializeComponents();
  /// Assignment of matrix indices for nodes
  void assignMatrixNodeIndices();
  /// Set apparent base power of per-unit system
  void setBaseApparentPower();
  /// Determine bus type for all buses
  void determinePFBusType();
  /// Rebuild index vectors + counts from the PQ/PV/VD node lists (after an
  /// initial classification or a Q-limit reclassification)
  void rebuildBusIndexAggregates();
  /// Re-derive index vectors and resize the system after PV<->PQ switching;
  /// the per-node solution vectors are preserved as a warm start
  void reclassifyBuses();
  /// Restore the original PV/PQ classification before a fresh solve (the solver
  /// is reused across timesteps)
  void resetToOriginalClassification();
  /// Clear Q-limit bookkeeping; overridden by PFSolverPowerPolar
  virtual void clearReactiveLimitState() {}
  /// Determine base voltages for each node
  void determineNodeBaseVoltages();

  /// Compose admittance matrix
  void composeAdmittanceMatrix();
  /// Gets the real part of admittance matrix element
  CPS::Real G(int i, int j);
  /// Gets the imaginary part of admittance matrix element
  CPS::Real B(int i, int j);
  /// Solves the powerflow problem
  Bool solvePowerflow();
  /// Run a single Newton-Raphson solve to convergence with the current bus
  /// classification (the inner loop of solvePowerflow)
  Bool runNewtonRaphson();
  /// Check generator reactive limits and switch violating PV buses to PQ (and
  /// relaxed pinned buses back to PV). Returns true if any bus switched. The
  /// base implementation is a no-op; PFSolverPowerPolar provides the logic.
  virtual CPS::Bool enforceReactiveLimits() { return false; }
  /// Allocate Jacobian storage; dense by default, sparse subclass overrides
  virtual void setUpJacobianStorage();
  /// Solve the linearized system mJ*mX = mF into mX; sparse subclass overrides
  virtual void solveJacobianSystem();
  /// Check whether below tolerance
  CPS::Bool checkConvergence();
  /// Logging for integer vectors
  CPS::String logVector(std::vector<CPS::UInt> indexVector) {
    std::stringstream result;
    std::copy(indexVector.begin(), indexVector.end(),
              std::ostream_iterator<CPS::UInt>(result, " "));
    return result.str();
  };
  ///
  CPS::Task::List getTasks() override;
  // determines power flow bus type for each node according to the components attached to it.
public:
  /// Constructor to be used in simulation examples.
  PFSolver(CPS::String name, CPS::SystemTopology system, Real timeStep,
           CPS::Logger::Level logLevel);
  ///
  virtual ~PFSolver(){};

  /// Set a node to VD using its name
  void setVDNode(CPS::String name);
  /// Allows to modify the powerflow bus type of a specific component
  void modifyPowerFlowBusComponent(CPS::String name,
                                   CPS::PowerflowBusType powerFlowBusType);
  /// set solver and component to initialization or simulation behaviour
  void setSolverAndComponentBehaviour(Solver::Behaviour behaviour) override;

  void setKeepLastSolution(CPS::Bool keepLastSolution) {
    mKeepLastSolution = keepLastSolution;
  }

  /// Enable generator reactive-limit enforcement (PV<->PQ outer loop)
  void setEnforceReactiveLimits(CPS::Bool value) {
    mEnforceReactiveLimits = value;
  }

  CPS::Bool getKeepLastSolution() const { return mKeepLastSolution; }

  class SolveTask : public CPS::Task {
  public:
    SolveTask(PFSolver &solver)
        : Task(solver.mName + ".Solve"), mSolver(solver) {
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount);

  private:
    PFSolver &mSolver;
  };
};
} // namespace DPsim
