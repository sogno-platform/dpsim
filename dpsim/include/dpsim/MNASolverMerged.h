/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
*                     EONERC, RWTH Aachen University
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*********************************************************************************/

#pragma once

#include <bitset>
#include <iostream>
#include <list>
#include <unordered_map>
#include <vector>

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNASyncGenInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim/Config.h>
#include <dpsim/DataLogger.h>
#include <dpsim/Solver.h>

/* std::size_t is the largest data type. No container can store
 * more than std::size_t elements. Define the number of switches
 * as the log_2 of this value so that we end up with maximally
 * std::size_t matrices. The overhead of statically defining this
 * value should be minimal.
 **/
#define SWITCH_NUM sizeof(std::size_t) * 8

namespace DPsim {
  enum DirectLinearSolverImpl {
  Undef = 0,
  KLU,
  SparseLU,
  DenseLU,
  CUDADense,
  CUDASparse,
  CUDAMagma,
  Plugin
};
/// Solver class using Modified Nodal Analysis (MNA).
template <typename VarType> class MnaSolverMerged : public Solver {
protected:
  // #### General simulation settings ####
  /// Simulation domain, which can be dynamic phasor (DP) or EMT
  CPS::Domain mDomain;
  /// Number of network and virtual nodes, single line equivalent
  UInt mNumNodes = 0;
  /// Number of network nodes, single line equivalent
  UInt mNumNetNodes = 0;
  /// Number of virtual nodes, single line equivalent
  UInt mNumVirtualNodes = 0;
  /// Number of network and virtual nodes, considering individual phases
  UInt mNumMatrixNodeIndices = 0;
  /// Number of network nodes, considering individual phases
  UInt mNumNetMatrixNodeIndices = 0;
  /// Number of virtual nodes, considering individual phases
  UInt mNumVirtualMatrixNodeIndices = 0;
  /// Number of nodes, excluding the primary frequency
  UInt mNumHarmMatrixNodeIndices = 0;
  /// Total number of network and virtual nodes, considering individual phases and additional frequencies
  UInt mNumTotalMatrixNodeIndices = 0;
  /// List of index pairs of varying matrix entries
  std::vector<std::pair<UInt, UInt>> mListVariableSystemMatrixEntries;

  /// System topology
  CPS::SystemTopology mSystem;
  /// List of simulation nodes
  typename CPS::SimNode<VarType>::List mNodes;

  // #### MNA specific attributes ####
  /// List of MNA components with static stamp into system matrix
  CPS::MNAInterface::List mMNAComponents;
  /// List of switches that stamp differently depending on their state
  /// and indicate the solver to choose a different system matrix
  CPS::MNASwitchInterface::List mSwitches;
  /// List of switches if they must be accessed as MNAInterface objects
  CPS::MNAInterface::List mMNAIntfSwitches;
  /// List of signal type components that do not directly interact with the MNA solver
  CPS::SimSignalComp::List mSimSignalComps;
  /// Current status of all switches encoded as bitset
  std::bitset<SWITCH_NUM> mCurrentSwitchStatus;
  /// List of synchronous generators that need iterate to solve the differential equations
  CPS::MNASyncGenInterface::List mSyncGen;

  /// Source vector of known quantities
  Matrix mRightSideVector;
  /// List of all right side vector contributions
  std::vector<const Matrix *> mRightVectorStamps;

  // #### MNA specific attributes related to harmonics / additional frequencies ####
  /// Source vector of known quantities
  std::vector<Matrix> mRightSideVectorHarm;

  // #### MNA specific attributes related to system recomputation
  /// Number of system matrix recomputations
  Int mNumRecomputations = 0;
  /// List of components that indicate the solver to recompute the system matrix
  /// depending on their state
  CPS::MNAVariableCompInterface::List mVariableComps;
  /// List of variable components if they must be accessed as MNAInterface objects
  CPS::MNAInterface::List mMNAIntfVariableComps;

  // #### Attributes related to switching ####
  /// Index of the next switching event
  UInt mSwitchTimeIndex = 0;
  /// Vector of switch times
  std::vector<SwitchConfiguration> mSwitchEvents;
  /// Collects the status of switches to select correct system matrix
  void updateSwitchStatus();

  // #### Attributes related to logging ####
  /// Last simulation time step when log was updated
  Int mLastLogTimeStep = 0;
  /// Left side vector logger
  std::shared_ptr<DataLogger> mLeftVectorLog;
  /// Right side vector logger
  std::shared_ptr<DataLogger> mRightVectorLog;

  /// LU factorization measurements
  std::vector<Real> mFactorizeTimes;
  /// Right-hand side solution measurements
  std::vector<Real> mSolveTimes;
  /// LU refactorization measurements
  std::vector<Real> mRecomputationTimes;

  // #### Data structures for precomputed switch matrices (optionally with parallel frequencies) ####
  /// Map of system matrices where the key is the bitset describing the switch states
  std::unordered_map<std::bitset<SWITCH_NUM>, std::vector<SparseMatrix>>
      mSwitchedMatrices;
  /// Map of direct linear solvers related to the system matrices
  std::unordered_map<std::bitset<SWITCH_NUM>,
                     std::vector<std::shared_ptr<DirectLinearSolver>>>
      mDirectLinearSolvers;

  // #### Data structures for system recomputation over time ####
  /// System matrix including all static elements
  SparseMatrix mBaseSystemMatrix;
  /// System matrix including stamp of static and variable elements
  SparseMatrix mVariableSystemMatrix;
  /// LU factorization of variable system matrix
  std::shared_ptr<DirectLinearSolver> mDirectLinearSolverVariableSystemMatrix;
  /// LU factorization indicator
  DirectLinearSolverImpl mImplementationInUse;
  /// LU factorization configuration
  DirectLinearSolverConfiguration mConfigurationInUse;


  /// Initialization of individual components
  void initializeComponents();
  /// Initialization of system matrices and source vector
  virtual void initializeSystem();
  /// Initialization of system matrices and source vector
  void initializeSystemWithParallelFrequencies();
  /// Initialization of system matrices and source vector
  void initializeSystemWithPrecomputedMatrices();
  /// Initialization of system matrices and source vector
  void initializeSystemWithVariableMatrix();
  /// Identify Nodes and SimPowerComps and SimSignalComps
  void identifyTopologyObjects();
  /// Assign simulation node index according to index in the vector.
  void assignMatrixNodeIndices();
  /// Collects virtual nodes inside components.
  /// The MNA algorithm handles these nodes in the same way as network nodes.
  void collectVirtualNodes();
  // TODO: check if this works with AC sources
  void steadyStateInitialization();

  /// Create left and right side vector
  void createEmptyVectors();
  /// Create system matrix
  void createEmptySystemMatrix();

  // #### Methods for precomputed switch matrices (optionally with parallel frequencies) ####
  /// Sets all entries in the matrix with the given switch index to zero
  void switchedMatrixEmpty(std::size_t index);
  /// Sets all entries in the matrix with the given switch index and frequency index to zero
  void switchedMatrixEmpty(std::size_t swIdx, Int freqIdx);
  /// Applies a component stamp to the matrix with the given switch index
  void switchedMatrixStamp(
      std::size_t index,
      std::vector<std::shared_ptr<CPS::MNAInterface>> &comp);
/// Applies a component and switch stamp to the matrix with the given switch index
  virtual void switchedMatrixStamp(std::size_t swIdx, Int freqIdx,
                                   CPS::MNAInterface::List &components,
                                   CPS::MNASwitchInterface::List &switches) {}

    /// Checks whether the status of variable MNA elements has changed
  Bool hasVariableComponentChanged();

// #### Methods for system recomputation over time ####
  /// Stamps components into the variable system matrix
  void stampVariableSystemMatrix();
  /// Solves the system with variable system matrix
  void solveWithSystemMatrixRecomputation(Real time,
                                          Int timeStepCount);
  /// Create a solve task for recomputation solver
  std::shared_ptr<CPS::Task> createSolveTaskRecomp();
  /// Recomputes systems matrix
  virtual void recomputeSystemMatrix(Real time);

  // #### Scheduler Task Methods ####
  /// Create a solve task for this solver implementation
  std::shared_ptr<CPS::Task> createSolveTask();
  /// Create a log task for this solver implementation
  std::shared_ptr<CPS::Task> createLogTask();
  /// Create a solve task for this solver implementation
  std::shared_ptr<CPS::Task> createSolveTaskHarm(UInt freqIdx);

  /// Logs left and right vector
  virtual void log(Real time, Int timeStepCount) override;
  /// Logging of system matrices and source vector
  void logSystemMatrices();
  /// Solves system for single frequency
  void solve(Real time, Int timeStepCount);
  /// Solves system for multiple frequencies
  void solveWithHarmonics(Real time, Int timeStepCount, Int freqIdx);

  /// Logging of the right-hand-side solution time
  void logSolveTime();
  /// Logging of the LU factorization time
  void logFactorizationTime();
  /// Logging of the LU refactorization time
  void logRecomputationTime();

  /// Returns a pointer to an object of type DirectLinearSolver
  std::shared_ptr<DirectLinearSolver>
  createDirectSolverImplementation(CPS::Logger::Log mSLog);

public:
  /// Constructor should not be called by users but by Simulation
  /// solverImpl: choose the most advanced solver implementation available by default
    MnaSolverMerged(String name, CPS::Domain domain = CPS::Domain::DP,
                  CPS::Logger::Level logLevel = CPS::Logger::Level::info);

  /// Destructor
  virtual ~MnaSolverMerged() {
    if (mSystemMatrixRecomputation)
      SPDLOG_LOGGER_INFO(mSLog, "Number of system matrix recomputations: {:}",
                         mNumRecomputations);
  };

  /// Solution vector of unknown quantities
  CPS::Attribute<Matrix>::Ptr mLeftSideVector;

  /// Solution vector of unknown quantities (parallel frequencies)
  std::vector<CPS::Attribute<Matrix>::Ptr> mLeftSideVectorHarm;

  /// Calls subroutines to set up everything that is required before simulation
  virtual void initialize() override;

  // #### Setter and Getter ####
  ///
  virtual void setSystem(const CPS::SystemTopology &system) override;
  ///
  Matrix &leftSideVector() { return **mLeftSideVector; }
  ///
  Matrix &rightSideVector() { return mRightSideVector; }
  ///
  virtual CPS::Task::List getTasks() override;

  /// Sets the linear solver to "implementation" and creates an object
  void
  setDirectLinearSolverImplementation(DirectLinearSolverImpl implementation);

  /// Sets the linear solver configuration
  void setDirectLinearSolverConfiguration(
      DirectLinearSolverConfiguration &configuration) override;

  /// log LU decomposition times
  void logLUTimes() override;

  /// ### SynGen Interface ###
  int mIter = 0;

 // #### MNA Solver Tasks ####
  ///
  class SolveTask : public CPS::Task {
  public:
    SolveTask(MnaSolverMerged<VarType> &solver)
        : Task(solver.mName + ".Solve"), mSolver(solver) {

      for (auto it : solver.mMNAComponents) {
        if (it->getRightVector()->get().size() != 0)
          mAttributeDependencies.push_back(it->getRightVector());
      }
      for (auto node : solver.mNodes) {
        mModifiedAttributes.push_back(node->mVoltage);
      }
      mModifiedAttributes.push_back(solver.mLeftSideVector);
    }

    void execute(Real time, Int timeStepCount) {
      mSolver.solve(time, timeStepCount);
    }

  private:
    MnaSolverMerged<VarType> &mSolver;
  };

  ///
  class SolveTaskHarm : public CPS::Task {
  public:
    SolveTaskHarm(MnaSolverMerged<VarType> &solver, UInt freqIdx)
        : Task(solver.mName + ".Solve"), mSolver(solver), mFreqIdx(freqIdx) {

      for (auto it : solver.mMNAComponents) {
        if (it->getRightVector()->get().size() != 0)
          mAttributeDependencies.push_back(it->getRightVector());
      }
      for (auto node : solver.mNodes) {
        mModifiedAttributes.push_back(node->mVoltage);
      }
      for (auto leftVec : solver.mLeftSideVectorHarm) {
        mModifiedAttributes.push_back(leftVec);
      }
    }

    void execute(Real time, Int timeStepCount) {
      mSolver.solveWithHarmonics(time, timeStepCount, mFreqIdx);
    }

  private:
    MnaSolverMerged<VarType> &mSolver;
    UInt mFreqIdx;
  };

  ///
  class SolveTaskRecomp : public CPS::Task {
  public:
    SolveTaskRecomp(MnaSolverMerged<VarType> &solver)
        : Task(solver.mName + ".Solve"), mSolver(solver) {

      for (auto it : solver.mMNAComponents) {
        if (it->getRightVector()->get().size() != 0)
          mAttributeDependencies.push_back(it->getRightVector());
      }
      for (auto it : solver.mMNAIntfVariableComps) {
        if (it->getRightVector()->get().size() != 0)
          mAttributeDependencies.push_back(it->getRightVector());
      }
      for (auto node : solver.mNodes) {
        mModifiedAttributes.push_back(node->mVoltage);
      }
      mModifiedAttributes.push_back(solver.mLeftSideVector);
    }

    void execute(Real time, Int timeStepCount) {
      mSolver.solveWithSystemMatrixRecomputation(time, timeStepCount);
      mSolver.log(time, timeStepCount);
    }

  private:
    MnaSolverMerged<VarType> &mSolver;
  };

  ///
  class LogTask : public CPS::Task {
  public:
    LogTask(MnaSolverMerged<VarType> &solver)
        : Task(solver.mName + ".Log"), mSolver(solver) {
      mAttributeDependencies.push_back(solver.mLeftSideVector);
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount) {
      mSolver.log(time, timeStepCount);
    }

  private:
    MnaSolverMerged<VarType> &mSolver;
  };
};
} // namespace DPsim
