/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverMerged.h>
#include <dpsim/SequentialScheduler.h>
#include <memory>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverMerged<VarType>::MnaSolverMerged(String name, CPS::Domain domain,
                              CPS::Logger::Level logLevel)
    : Solver(name, logLevel), mDomain(domain) {
  mImplementationInUse = DirectLinearSolverImpl::KLU;
  // Raw source and solution vector logging
  mLeftVectorLog = std::make_shared<DataLogger>(
      name + "_LeftVector", logLevel == CPS::Logger::Level::trace);
  mRightVectorLog = std::make_shared<DataLogger>(
      name + "_RightVector", logLevel == CPS::Logger::Level::trace);
}

template <typename VarType>
void MnaSolverMerged<VarType>::setSystem(const CPS::SystemTopology &system) {
  mSystem = system;
}

template <typename VarType> void MnaSolverMerged<VarType>::initialize() {
  // TODO: check that every system matrix has the same dimensions
  SPDLOG_LOGGER_INFO(mSLog, "---- Start initialization ----");

  // Register attribute for solution vector
  ///FIXME: This is kinda ugly... At least we should somehow unify mLeftSideVector and mLeftSideVectorHarm.
  // Best case we have some kind of sub-attributes for attribute vectors / tensor attributes...
  if (mFrequencyParallel) {
    SPDLOG_LOGGER_INFO(mSLog, "Computing network harmonics in parallel.");
    for (Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
      mLeftSideVectorHarm.push_back(AttributeStatic<Matrix>::make());
    }
  } else {
    mLeftSideVector = AttributeStatic<Matrix>::make();
  }

  SPDLOG_LOGGER_INFO(mSLog, "-- Process topology");
  for (auto comp : mSystem.mComponents)
    SPDLOG_LOGGER_INFO(mSLog, "Added {:s} '{:s}' to simulation.", comp->type(),
                       comp->name());

  // Otherwise LU decomposition will fail
  if (mSystem.mComponents.size() == 0)
    throw SolverException();

  // We need to differentiate between power and signal components and
  // ground nodes should be ignored.
  identifyTopologyObjects();
  // These steps complete the network information.
  collectVirtualNodes();
  assignMatrixNodeIndices();

  SPDLOG_LOGGER_INFO(mSLog, "-- Create empty MNA system matrices and vectors");
  createEmptyVectors();
  createEmptySystemMatrix();

  // Initialize components from powerflow solution and
  // calculate MNA specific initialization values.
  initializeComponents();

  if (mSteadyStateInit) {
    mIsInInitialization = true;
    steadyStateInitialization();
  }
  mIsInInitialization = false;

  // Some components feature a different behaviour for simulation and initialization
  for (auto comp : mSystem.mComponents) {
    auto powerComp = std::dynamic_pointer_cast<CPS::TopologicalPowerComp>(comp);
    if (powerComp)
      powerComp->setBehaviour(TopologicalPowerComp::Behaviour::MNASimulation);

    auto sigComp = std::dynamic_pointer_cast<CPS::SimSignalComp>(comp);
    if (sigComp)
      sigComp->setBehaviour(SimSignalComp::Behaviour::Simulation);
  }

  // Initialize system matrices and source vector.
  initializeSystem();

  SPDLOG_LOGGER_INFO(mSLog, "--- Initialization finished ---");
  SPDLOG_LOGGER_INFO(mSLog, "--- Initial system matrices and vectors ---");
  logSystemMatrices();

  mSLog->flush();
}

template <> void MnaSolverMerged<Real>::initializeComponents() {
  SPDLOG_LOGGER_INFO(mSLog, "-- Initialize components from power flow");

  CPS::MNAInterface::List allMNAComps;
  allMNAComps.insert(allMNAComps.end(), mMNAComponents.begin(),
                     mMNAComponents.end());
  allMNAComps.insert(allMNAComps.end(), mMNAIntfVariableComps.begin(),
                     mMNAIntfVariableComps.end());

  for (auto comp : allMNAComps) {
    auto pComp = std::dynamic_pointer_cast<SimPowerComp<Real>>(comp);
    if (!pComp)
      continue;
    pComp->checkForUnconnectedTerminals();
    if (mInitFromNodesAndTerminals)
      pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
  }

  // Initialize signal components.
  for (auto comp : mSimSignalComps)
    comp->initialize(mSystem.mSystemOmega, mTimeStep);

  // Initialize MNA specific parts of components.
  for (auto comp : allMNAComps) {
    comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);
    const Matrix &stamp = comp->getRightVector()->get();
    if (stamp.size() != 0) {
      mRightVectorStamps.push_back(&stamp);
    }
  }

  for (auto comp : mMNAIntfSwitches)
    comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);

  // Initialize nodes
  for (UInt nodeIdx = 0; nodeIdx < mNodes.size(); ++nodeIdx)
    mNodes[nodeIdx]->initialize();
}

template <> void MnaSolverMerged<Complex>::initializeComponents() {
  SPDLOG_LOGGER_INFO(mSLog, "-- Initialize components from power flow");

  CPS::MNAInterface::List allMNAComps;
  allMNAComps.insert(allMNAComps.end(), mMNAComponents.begin(),
                     mMNAComponents.end());
  allMNAComps.insert(allMNAComps.end(), mMNAIntfVariableComps.begin(),
                     mMNAIntfVariableComps.end());

  // Initialize power components with frequencies and from powerflow results
  for (auto comp : allMNAComps) {
    auto pComp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp);
    if (!pComp)
      continue;
    pComp->checkForUnconnectedTerminals();
    if (mInitFromNodesAndTerminals)
      pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
  }

  // Initialize signal components.
  for (auto comp : mSimSignalComps)
    comp->initialize(mSystem.mSystemOmega, mTimeStep);

  SPDLOG_LOGGER_INFO(mSLog, "-- Initialize MNA properties of components");
  if (mFrequencyParallel) {
    // Initialize MNA specific parts of components.
    for (auto comp : mMNAComponents) {
      // Initialize MNA specific parts of components.
      comp->mnaInitializeHarm(mSystem.mSystemOmega, mTimeStep,
                              mLeftSideVectorHarm);
      const Matrix &stamp = comp->getRightVector()->get();
      if (stamp.size() != 0)
        mRightVectorStamps.push_back(&stamp);
    }
    // Initialize nodes
    for (UInt nodeIdx = 0; nodeIdx < mNodes.size(); ++nodeIdx) {
      mNodes[nodeIdx]->mnaInitializeHarm(mLeftSideVectorHarm);
    }
  } else {
    // Initialize MNA specific parts of components.
    for (auto comp : allMNAComps) {
      comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);
      const Matrix &stamp = comp->getRightVector()->get();
      if (stamp.size() != 0) {
        mRightVectorStamps.push_back(&stamp);
      }
    }

    for (auto comp : mMNAIntfSwitches)
      comp->mnaInitialize(mSystem.mSystemOmega, mTimeStep, mLeftSideVector);

    // Initialize nodes
    for (UInt nodeIdx = 0; nodeIdx < mNodes.size(); ++nodeIdx)
      mNodes[nodeIdx]->initialize();
  }
}

template <typename VarType> void MnaSolverMerged<VarType>::initializeSystem() {
  SPDLOG_LOGGER_INFO(mSLog,
                     "-- Initialize MNA system matrices and source vector");
  mRightSideVector.setZero();

  // just a sanity check in case we change the static
  // initialization of the switch number in the future
  if (mSwitches.size() > sizeof(std::size_t) * 8) {
    throw SystemError("Too many Switches.");
  }

  if (mFrequencyParallel)
    initializeSystemWithParallelFrequencies();
  else if (mSystemMatrixRecomputation)
    initializeSystemWithVariableMatrix();
  else
    initializeSystemWithPrecomputedMatrices();
}

template <typename VarType>
void MnaSolverMerged<VarType>::initializeSystemWithParallelFrequencies() {
  // iterate over all possible switch state combinations and frequencies
  for (std::size_t sw = 0; sw < (1ULL << mSwitches.size()); ++sw) {
    for (Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
      switchedMatrixEmpty(sw, freq);
      switchedMatrixStamp(sw, freq, mMNAComponents, mSwitches);
    }
  }

  if (mSwitches.size() > 0)
    updateSwitchStatus();

  // Initialize source vector
  for (Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
    for (auto comp : mMNAComponents)
      comp->mnaApplyRightSideVectorStampHarm(mRightSideVectorHarm[freq], freq);
  }
}

template <typename VarType>
void MnaSolverMerged<VarType>::initializeSystemWithPrecomputedMatrices() {
  // iterate over all possible switch state combinations
  for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
    switchedMatrixEmpty(i);
  }

  if (mSwitches.size() < 1) {
    switchedMatrixStamp(0, mMNAComponents);
  } else {
    // Generate switching state dependent system matrices
    for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
      switchedMatrixStamp(i, mMNAComponents);
    }
    updateSwitchStatus();
  }

  // Initialize source vector for debugging
  // CAUTION: this does not always deliver proper source vector initialization
  // as not full pre-step is executed (not involving necessary electrical or signal
  // subcomp updates before right vector calculation)
  for (auto comp : mMNAComponents) {
    comp->mnaApplyRightSideVectorStamp(mRightSideVector);
    auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
    SPDLOG_LOGGER_DEBUG(mSLog, "Stamping {:s} {:s} into source vector",
                        idObj->type(), idObj->name());
    if (mSLog->should_log(spdlog::level::trace))
      mSLog->trace("\n{:s}", Logger::matrixToString(mRightSideVector));
  }
}

template <typename VarType>
void MnaSolverMerged<VarType>::initializeSystemWithVariableMatrix() {

  // Collect index pairs of varying matrix entries from components
  for (auto varElem : mVariableComps)
    for (auto varEntry : varElem->mVariableSystemMatrixEntries)
      mListVariableSystemMatrixEntries.push_back(varEntry);
  SPDLOG_LOGGER_INFO(mSLog, "List of index pairs of varying matrix entries: ");
  for (auto indexPair : mListVariableSystemMatrixEntries)
    SPDLOG_LOGGER_INFO(mSLog, "({}, {})", indexPair.first, indexPair.second);

  stampVariableSystemMatrix();

  // Initialize source vector for debugging
  // CAUTION: this does not always deliver proper source vector initialization
  // as not full pre-step is executed (not involving necessary electrical or signal
  // subcomp updates before right vector calculation)
  for (auto comp : mMNAComponents) {
    comp->mnaApplyRightSideVectorStamp(mRightSideVector);
    auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(comp);
    SPDLOG_LOGGER_DEBUG(mSLog, "Stamping {:s} {:s} into source vector",
                        idObj->type(), idObj->name());
    if (mSLog->should_log(spdlog::level::trace))
      mSLog->trace("\n{:s}", Logger::matrixToString(mRightSideVector));
  }
}

template <typename VarType>
Bool MnaSolverMerged<VarType>::hasVariableComponentChanged() {
  for (auto varElem : mVariableComps) {
    if (varElem->hasParameterChanged()) {
      auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(varElem);
      SPDLOG_LOGGER_DEBUG(
          mSLog, "Component ({:s} {:s}) value changed -> Update System Matrix",
          idObj->type(), idObj->name());
      return true;
    }
  }
  return false;
}

template <typename VarType> void MnaSolverMerged<VarType>::updateSwitchStatus() {
  for (UInt i = 0; i < mSwitches.size(); ++i) {
    mCurrentSwitchStatus.set(i, mSwitches[i]->mnaIsClosed());
  }
}

template <typename VarType> void MnaSolverMerged<VarType>::identifyTopologyObjects() {
  for (auto baseNode : mSystem.mNodes) {
    // Add nodes to the list and ignore ground nodes.
    if (!baseNode->isGround()) {
      auto node = std::dynamic_pointer_cast<CPS::SimNode<VarType>>(baseNode);
      mNodes.push_back(node);
      SPDLOG_LOGGER_INFO(mSLog, "Added node {:s}", node->name());
    }
  }

  for (auto comp : mSystem.mComponents) {

    auto genComp = std::dynamic_pointer_cast<CPS::MNASyncGenInterface>(comp);
    if (genComp) {
      mSyncGen.push_back(genComp);
    }

    auto swComp = std::dynamic_pointer_cast<CPS::MNASwitchInterface>(comp);
    if (swComp) {
      mSwitches.push_back(swComp);
      auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(swComp);
      if (mnaComp)
        mMNAIntfSwitches.push_back(mnaComp);
    }

    auto varComp =
        std::dynamic_pointer_cast<CPS::MNAVariableCompInterface>(comp);
    if (varComp) {
      mVariableComps.push_back(varComp);
      auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(varComp);
      if (mnaComp)
        mMNAIntfVariableComps.push_back(mnaComp);
    }

    if (!(swComp || varComp)) {
      auto mnaComp = std::dynamic_pointer_cast<CPS::MNAInterface>(comp);
      if (mnaComp)
        mMNAComponents.push_back(mnaComp);

      auto sigComp = std::dynamic_pointer_cast<CPS::SimSignalComp>(comp);
      if (sigComp)
        mSimSignalComps.push_back(sigComp);
    }
  }
}

template <typename VarType> void MnaSolverMerged<VarType>::assignMatrixNodeIndices() {
  UInt matrixNodeIndexIdx = 0;
  for (UInt idx = 0; idx < mNodes.size(); ++idx) {
    mNodes[idx]->setMatrixNodeIndex(0, matrixNodeIndexIdx);
    SPDLOG_LOGGER_INFO(mSLog, "Assigned index {} to phase A of node {}",
                       matrixNodeIndexIdx, idx);
    ++matrixNodeIndexIdx;
    if (mNodes[idx]->phaseType() == CPS::PhaseType::ABC) {
      mNodes[idx]->setMatrixNodeIndex(1, matrixNodeIndexIdx);
      SPDLOG_LOGGER_INFO(mSLog, "Assigned index {} to phase B of node {}",
                         matrixNodeIndexIdx, idx);
      ++matrixNodeIndexIdx;
      mNodes[idx]->setMatrixNodeIndex(2, matrixNodeIndexIdx);
      SPDLOG_LOGGER_INFO(mSLog, "Assigned index {} to phase C of node {}",
                         matrixNodeIndexIdx, idx);
      ++matrixNodeIndexIdx;
    }
    // This should be true when the final network node is reached, not considering virtual nodes
    if (idx == mNumNetNodes - 1)
      mNumNetMatrixNodeIndices = matrixNodeIndexIdx;
  }
  // Total number of network nodes including virtual nodes is matrixNodeIndexIdx + 1, which is why the variable is incremented after assignment
  mNumMatrixNodeIndices = matrixNodeIndexIdx;
  mNumVirtualMatrixNodeIndices =
      mNumMatrixNodeIndices - mNumNetMatrixNodeIndices;
  mNumHarmMatrixNodeIndices =
      static_cast<UInt>(mSystem.mFrequencies.size() - 1) *
      mNumMatrixNodeIndices;
  mNumTotalMatrixNodeIndices =
      static_cast<UInt>(mSystem.mFrequencies.size()) * mNumMatrixNodeIndices;

  SPDLOG_LOGGER_INFO(mSLog, "Assigned simulation nodes to topology nodes:");
  SPDLOG_LOGGER_INFO(mSLog, "Number of network simulation nodes: {:d}",
                     mNumNetMatrixNodeIndices);
  SPDLOG_LOGGER_INFO(mSLog, "Number of simulation nodes: {:d}",
                     mNumMatrixNodeIndices);
  SPDLOG_LOGGER_INFO(mSLog, "Number of harmonic simulation nodes: {:d}",
                     mNumHarmMatrixNodeIndices);
}

template <> void MnaSolverMerged<Real>::createEmptyVectors() {
  mRightSideVector = Matrix::Zero(mNumMatrixNodeIndices, 1);
  **mLeftSideVector = Matrix::Zero(mNumMatrixNodeIndices, 1);
}

template <> void MnaSolverMerged<Complex>::createEmptyVectors() {
  if (mFrequencyParallel) {
    for (Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
      mRightSideVectorHarm.push_back(
          Matrix::Zero(2 * (mNumMatrixNodeIndices), 1));
      mLeftSideVectorHarm.push_back(AttributeStatic<Matrix>::make(
          Matrix::Zero(2 * (mNumMatrixNodeIndices), 1)));
    }
  } else {
    mRightSideVector = Matrix::Zero(
        2 * (mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 1);
    **mLeftSideVector = Matrix::Zero(
        2 * (mNumMatrixNodeIndices + mNumHarmMatrixNodeIndices), 1);
  }
}

template <typename VarType> void MnaSolverMerged<VarType>::collectVirtualNodes() {
  // We have not added virtual nodes yet so the list has only network nodes
  mNumNetNodes = (UInt)mNodes.size();
  // virtual nodes are placed after network nodes
  UInt virtualNode = mNumNetNodes - 1;

  for (auto comp : mMNAComponents) {
    auto pComp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp);
    if (!pComp)
      continue;

    // Check if component requires virtual node and if so get a reference
    if (pComp->hasVirtualNodes()) {
      for (UInt node = 0; node < pComp->virtualNodesNumber(); ++node) {
        mNodes.push_back(pComp->virtualNode(node));
        SPDLOG_LOGGER_INFO(mSLog, "Collected virtual node {} of {}",
                           virtualNode, node, pComp->name());
      }
    }

    // Repeat the same steps for virtual nodes of sub components
    // TODO: recursive behavior
    if (pComp->hasSubComponents()) {
      for (auto pSubComp : pComp->subComponents()) {
        for (UInt node = 0; node < pSubComp->virtualNodesNumber(); ++node) {
          mNodes.push_back(pSubComp->virtualNode(node));
          SPDLOG_LOGGER_INFO(mSLog, "Collected virtual node {} of {}",
                             virtualNode, node, pComp->name());
        }
      }
    }
  }

  // collect virtual nodes of variable components
  for (auto comp : mVariableComps) {
    auto pComp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp);
    if (!pComp)
      continue;

    // Check if component requires virtual node and if so get a reference
    if (pComp->hasVirtualNodes()) {
      for (UInt node = 0; node < pComp->virtualNodesNumber(); ++node) {
        mNodes.push_back(pComp->virtualNode(node));
        SPDLOG_LOGGER_INFO(mSLog,
                           "Collected virtual node {} of Varible Comp {}", node,
                           pComp->name());
      }
    }
  }

  // Update node number to create matrices and vectors
  mNumNodes = (UInt)mNodes.size();
  mNumVirtualNodes = mNumNodes - mNumNetNodes;
  SPDLOG_LOGGER_INFO(mSLog, "Created virtual nodes:");
  SPDLOG_LOGGER_INFO(mSLog, "Number of network nodes: {:d}", mNumNetNodes);
  SPDLOG_LOGGER_INFO(mSLog, "Number of network and virtual nodes: {:d}",
                     mNumNodes);
}

template <typename VarType>
void MnaSolverMerged<VarType>::steadyStateInitialization() {
  SPDLOG_LOGGER_INFO(mSLog, "--- Run steady-state initialization ---");

  DataLogger initLeftVectorLog(mName + "_InitLeftVector",
                               mLogLevel != CPS::Logger::Level::off);
  DataLogger initRightVectorLog(mName + "_InitRightVector",
                                mLogLevel != CPS::Logger::Level::off);

  TopologicalPowerComp::Behaviour initBehaviourPowerComps =
      TopologicalPowerComp::Behaviour::Initialization;
  SimSignalComp::Behaviour initBehaviourSignalComps =
      SimSignalComp::Behaviour::Initialization;

  // TODO: enable use of timestep distinct from simulation timestep
  Real initTimeStep = mTimeStep;

  Int timeStepCount = 0;
  Real time = 0;
  Real maxDiff = 1.0;
  Real max = 1.0;
  Matrix diff = Matrix::Zero(2 * mNumNodes, 1);
  Matrix prevLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Time step is {:f}s for steady-state initialization",
                     initTimeStep);

  for (auto comp : mSystem.mComponents) {
    auto powerComp = std::dynamic_pointer_cast<CPS::TopologicalPowerComp>(comp);
    if (powerComp)
      powerComp->setBehaviour(initBehaviourPowerComps);

    auto sigComp = std::dynamic_pointer_cast<CPS::SimSignalComp>(comp);
    if (sigComp)
      sigComp->setBehaviour(initBehaviourSignalComps);
  }

  initializeSystem();
  logSystemMatrices();

  // Use sequential scheduler
  SequentialScheduler sched;
  CPS::Task::List tasks;
  Scheduler::Edges inEdges, outEdges;

  for (auto node : mNodes) {
    for (auto task : node->mnaTasks())
      tasks.push_back(task);
  }
  for (auto comp : mMNAComponents) {
    for (auto task : comp->mnaTasks()) {
      tasks.push_back(task);
    }
  }
  // TODO signal components should be moved out of MNA solver
  for (auto comp : mSimSignalComps) {
    for (auto task : comp->getTasks()) {
      tasks.push_back(task);
    }
  }
  tasks.push_back(createSolveTask());

  sched.resolveDeps(tasks, inEdges, outEdges);
  sched.createSchedule(tasks, inEdges, outEdges);

  while (time < mSteadStIniTimeLimit) {
    // Reset source vector
    mRightSideVector.setZero();

    sched.step(time, timeStepCount);

    if (mDomain == CPS::Domain::EMT) {
      initLeftVectorLog.logEMTNodeValues(time, leftSideVector());
      initRightVectorLog.logEMTNodeValues(time, rightSideVector());
    } else {
      initLeftVectorLog.logPhasorNodeValues(time, leftSideVector());
      initRightVectorLog.logPhasorNodeValues(time, rightSideVector());
    }

    // Calculate new simulation time
    time = time + initTimeStep;
    ++timeStepCount;

    // Calculate difference
    diff = prevLeftSideVector - **mLeftSideVector;
    prevLeftSideVector = **mLeftSideVector;
    maxDiff = diff.lpNorm<Eigen::Infinity>();
    max = (**mLeftSideVector).lpNorm<Eigen::Infinity>();
    // If difference is smaller than some epsilon, break
    if ((maxDiff / max) < mSteadStIniAccLimit)
      break;
  }

  SPDLOG_LOGGER_INFO(mSLog, "Max difference: {:f} or {:f}% at time {:f}",
                     maxDiff, maxDiff / max, time);

  // Reset system for actual simulation
  mRightSideVector.setZero();

  SPDLOG_LOGGER_INFO(mSLog, "--- Finished steady-state initialization ---");
}

template <typename VarType> Task::List MnaSolverMerged<VarType>::getTasks() {
  Task::List l;

  for (auto comp : mMNAComponents) {
    for (auto task : comp->mnaTasks()) {
      l.push_back(task);
    }
  }
  for (auto comp : mMNAIntfSwitches) {
    for (auto task : comp->mnaTasks()) {
      l.push_back(task);
    }
  }
  for (auto node : mNodes) {
    for (auto task : node->mnaTasks())
      l.push_back(task);
  }
  // TODO signal components should be moved out of MNA solver
  for (auto comp : mSimSignalComps) {
    for (auto task : comp->getTasks()) {
      l.push_back(task);
    }
  }
  if (mFrequencyParallel) {
    for (UInt i = 0; i < mSystem.mFrequencies.size(); ++i)
      l.push_back(createSolveTaskHarm(i));
  } else if (mSystemMatrixRecomputation) {
    for (auto comp : this->mMNAIntfVariableComps) {
      for (auto task : comp->mnaTasks())
        l.push_back(task);
    }
    l.push_back(createSolveTaskRecomp());
  } else {
    l.push_back(createSolveTask());
    l.push_back(createLogTask());
  }
  return l;
}

template <typename VarType>
void MnaSolverMerged<VarType>::log(Real time, Int timeStepCount) {
  if (mLogLevel == Logger::Level::off)
    return;

  if (mDomain == CPS::Domain::EMT) {
    mLeftVectorLog->logEMTNodeValues(time, leftSideVector());
    mRightVectorLog->logEMTNodeValues(time, rightSideVector());
  } else {
    mLeftVectorLog->logPhasorNodeValues(time, leftSideVector());
    mRightVectorLog->logPhasorNodeValues(time, rightSideVector());
  }
}

template <typename VarType>
void MnaSolverMerged<VarType>::switchedMatrixEmpty(std::size_t index) {
  mSwitchedMatrices[std::bitset<SWITCH_NUM>(index)][0].setZero();
}

template <typename VarType>
void MnaSolverMerged<VarType>::switchedMatrixEmpty(std::size_t swIdx,
                                                   Int freqIdx) {
  mSwitchedMatrices[std::bitset<SWITCH_NUM>(swIdx)][freqIdx].setZero();
}

template <typename VarType>
void MnaSolverMerged<VarType>::switchedMatrixStamp(
    std::size_t index, std::vector<std::shared_ptr<CPS::MNAInterface>> &comp) {
  auto bit = std::bitset<SWITCH_NUM>(index);
  auto &sys = mSwitchedMatrices[bit][0];
  for (auto component : comp) {
    component->mnaApplySystemMatrixStamp(sys);
  }
  for (UInt i = 0; i < mSwitches.size(); ++i)
    mSwitches[i]->mnaApplySwitchSystemMatrixStamp(bit[i], sys, 0);

  // Compute LU-factorization for system matrix
  mDirectLinearSolvers[bit][0]->preprocessing(sys,
                                              mListVariableSystemMatrixEntries);
  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolvers[bit][0]->factorize(sys);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mFactorizeTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverMerged<VarType>::stampVariableSystemMatrix() {

  this->mDirectLinearSolverVariableSystemMatrix =
      createDirectSolverImplementation(mSLog);
  // TODO: a direct linear solver configuration is only applied if system matrix recomputation is used
  this->mDirectLinearSolverVariableSystemMatrix->setConfiguration(
      mConfigurationInUse);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Number of variable Elements: {}"
                     "\nNumber of MNA components: {}",
                     mVariableComps.size(), mMNAComponents.size());

  // Build base matrix with only static elements
  mBaseSystemMatrix.setZero();
  for (auto statElem : mMNAComponents)
    statElem->mnaApplySystemMatrixStamp(mBaseSystemMatrix);
  SPDLOG_LOGGER_INFO(mSLog, "Base matrix with only static elements: {}",
                     Logger::matrixToString(mBaseSystemMatrix));
  mSLog->flush();

  // Continue from base matrix
  mVariableSystemMatrix = mBaseSystemMatrix;

  // Now stamp switches into matrix
  SPDLOG_LOGGER_INFO(mSLog, "Stamping switches");
  for (auto sw : mMNAIntfSwitches)
    sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  // Now stamp initial state of variable elements into matrix
  SPDLOG_LOGGER_INFO(mSLog, "Stamping variable elements");
  for (auto varElem : mMNAIntfVariableComps)
    varElem->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  SPDLOG_LOGGER_INFO(mSLog, "Initial system matrix with variable elements {}",
                     Logger::matrixToString(mVariableSystemMatrix));
  /* TODO: find replacement for flush() */
  mSLog->flush();

  // Calculate factorization of current matrix
  mDirectLinearSolverVariableSystemMatrix->preprocessing(
      mVariableSystemMatrix, mListVariableSystemMatrixEntries);

  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolverVariableSystemMatrix->factorize(mVariableSystemMatrix);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mFactorizeTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverMerged<VarType>::solveWithSystemMatrixRecomputation(
    Real time, Int timeStepCount) {
  // Reset source vector
  mRightSideVector.setZero();

  // Add together the right side vector (computed by the components'
  // pre-step tasks)
  for (auto stamp : mRightVectorStamps)
    mRightSideVector += *stamp;

  // Get switch and variable comp status and update system matrix and lu factorization accordingly
  if (hasVariableComponentChanged())
    recomputeSystemMatrix(time);

  // Calculate new solution vector
  auto start = std::chrono::steady_clock::now();
  **mLeftSideVector =
      mDirectLinearSolverVariableSystemMatrix->solve(mRightSideVector);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mSolveTimes.push_back(diff.count());

  // TODO split into separate task? (dependent on x, updating all v attributes)
  for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
    mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

  // Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverMerged<VarType>::recomputeSystemMatrix(Real time) {
  // Start from base matrix
  mVariableSystemMatrix = mBaseSystemMatrix;

  // Now stamp switches into matrix
  for (auto sw : mMNAIntfSwitches)
    sw->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  // Now stamp variable elements into matrix
  for (auto comp : mMNAIntfVariableComps)
    comp->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  // Refactorization of matrix assuming that structure remained
  // constant by omitting analyzePattern
  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolverVariableSystemMatrix->partialRefactorize(
      mVariableSystemMatrix, mListVariableSystemMatrixEntries);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mRecomputationTimes.push_back(diff.count());
  ++mNumRecomputations;
}

template <> void MnaSolverMerged<Real>::createEmptySystemMatrix() {
  if (mSwitches.size() > SWITCH_NUM)
    throw SystemError("Too many Switches.");

  if (mSystemMatrixRecomputation) {
    mBaseSystemMatrix =
        SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
    mVariableSystemMatrix =
        SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
  } else {
    for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
      auto bit = std::bitset<SWITCH_NUM>(i);
      mSwitchedMatrices[bit].push_back(
          SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices));
      mDirectLinearSolvers[bit].push_back(
          createDirectSolverImplementation(mSLog));
    }
  }
}

template <> void MnaSolverMerged<Complex>::createEmptySystemMatrix() {
  if (mSwitches.size() > SWITCH_NUM)
    throw SystemError("Too many Switches.");

  if (mFrequencyParallel) {
    for (UInt i = 0; i < std::pow(2, mSwitches.size()); ++i) {
      for (Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
        auto bit = std::bitset<SWITCH_NUM>(i);
        mSwitchedMatrices[bit].push_back(SparseMatrix(
            2 * (mNumMatrixNodeIndices), 2 * (mNumMatrixNodeIndices)));
        mDirectLinearSolvers[bit].push_back(
            createDirectSolverImplementation(mSLog));
      }
    }
  } else if (mSystemMatrixRecomputation) {
    mBaseSystemMatrix =
        SparseMatrix(2 * (mNumMatrixNodeIndices), 2 * (mNumMatrixNodeIndices));
    mVariableSystemMatrix =
        SparseMatrix(2 * (mNumMatrixNodeIndices), 2 * (mNumMatrixNodeIndices));
  } else {
    for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
      auto bit = std::bitset<SWITCH_NUM>(i);
      mSwitchedMatrices[bit].push_back(SparseMatrix(
          2 * (mNumTotalMatrixNodeIndices), 2 * (mNumTotalMatrixNodeIndices)));
      mDirectLinearSolvers[bit].push_back(
          createDirectSolverImplementation(mSLog));
    }
  }
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverMerged<VarType>::createSolveTask() {
  return std::make_shared<MnaSolverMerged<VarType>::SolveTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverMerged<VarType>::createSolveTaskRecomp() {
  return std::make_shared<MnaSolverMerged<VarType>::SolveTaskRecomp>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task>
MnaSolverMerged<VarType>::createSolveTaskHarm(UInt freqIdx) {
  return std::make_shared<MnaSolverMerged<VarType>::SolveTaskHarm>(*this,
                                                                   freqIdx);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverMerged<VarType>::createLogTask() {
  return std::make_shared<MnaSolverMerged<VarType>::LogTask>(*this);
}

template <typename VarType>
void MnaSolverMerged<VarType>::solve(Real time, Int timeStepCount) {
  // Reset source vector
  mRightSideVector.setZero();

  // Add together the right side vector (computed by the components' pre-step tasks)
  for (auto stamp : mRightVectorStamps)
    mRightSideVector += *stamp;

  if (!mIsInInitialization)
    MnaSolver<VarType>::updateSwitchStatus();

  if (mSwitchedMatrices.size() > 0) {
    auto start = std::chrono::steady_clock::now();
    **mLeftSideVector =
        mDirectLinearSolvers[mCurrentSwitchStatus][0]->solve(mRightSideVector);
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<Real> diff = end - start;
    mSolveTimes.push_back(diff.count());
  }

  // CHECK: Is this really required? Or can operations actually become part of
  // correctorStep and mnaPostStep?
  for (auto syncGen : mSyncGen)
    syncGen->updateVoltage(**mLeftSideVector);

  // Reset number of iterations
  mIter = 0;

  // Additional solve steps for iterative models
  if (mSyncGen.size() > 0) {
    UInt numCompsRequireIter;
    do {
      // count synchronous generators that require iteration
      numCompsRequireIter = 0;
      for (auto syncGen : mSyncGen)
        if (syncGen->requiresIteration())
          numCompsRequireIter++;

      // recompute solve step if at least one component demands iteration
      if (numCompsRequireIter > 0) {
        mIter++;

        // Reset source vector
        mRightSideVector.setZero();

        if (!mIsInInitialization)
          MnaSolver<VarType>::updateSwitchStatus();

        for (auto syncGen : mSyncGen)
          syncGen->correctorStep();

        // Add together the right side vector (computed by the components' pre-step tasks)
        for (auto stamp : mRightVectorStamps)
          mRightSideVector += *stamp;

        if (mSwitchedMatrices.size() > 0) {
          auto start = std::chrono::steady_clock::now();
          **mLeftSideVector =
              mDirectLinearSolvers[mCurrentSwitchStatus][0]->solve(
                  mRightSideVector);
          auto end = std::chrono::steady_clock::now();
          std::chrono::duration<Real> diff = end - start;
          mSolveTimes.push_back(diff.count());
        }

        // CHECK: Is this really required? Or can operations actually become part of
        // correctorStep and mnaPostStep?
        for (auto syncGen : mSyncGen)
          syncGen->updateVoltage(**mLeftSideVector);
      }
    } while (numCompsRequireIter > 0);
  }

  // TODO split into separate task? (dependent on x, updating all v attributes)
  for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
    mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

  // Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverMerged<VarType>::solveWithHarmonics(Real time, Int timeStepCount,
                                                  Int freqIdx) {
  mRightSideVectorHarm[freqIdx].setZero();

  // Sum of right side vectors (computed by the components' pre-step tasks)
  for (auto stamp : mRightVectorStamps)
    mRightSideVectorHarm[freqIdx] += stamp->col(freqIdx);

  **mLeftSideVectorHarm[freqIdx] =
      mDirectLinearSolvers[mCurrentSwitchStatus][freqIdx]->solve(
          mRightSideVectorHarm[freqIdx]);
}

template <typename VarType> void MnaSolverMerged<VarType>::logSystemMatrices() {
  if (mFrequencyParallel) {
    for (UInt i = 0; i < mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)].size();
         ++i) {
      SPDLOG_LOGGER_INFO(mSLog, "System matrix for frequency: {:d} \n{:s}", i,
                         Logger::matrixToString(
                             mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][i]));
    }

    for (UInt i = 0; i < mRightSideVectorHarm.size(); ++i) {
      SPDLOG_LOGGER_INFO(mSLog, "Right side vector for frequency: {:d} \n{:s}",
                         i, Logger::matrixToString(mRightSideVectorHarm[i]));
    }

  } else if (mSystemMatrixRecomputation) {
    SPDLOG_LOGGER_INFO(mSLog, "Summarizing matrices: ");
    SPDLOG_LOGGER_INFO(mSLog, "Base matrix with only static elements: {}",
                       Logger::matrixToString(mBaseSystemMatrix));
    SPDLOG_LOGGER_INFO(mSLog, "Initial system matrix with variable elements {}",
                       Logger::matrixToString(mVariableSystemMatrix));
    SPDLOG_LOGGER_INFO(mSLog, "Right side vector: {}",
                       Logger::matrixToString(mRightSideVector));
  } else {
    if (mSwitches.size() < 1) {
      SPDLOG_LOGGER_INFO(mSLog, "System matrix: \n{}",
                         mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][0]);
    } else {
      SPDLOG_LOGGER_INFO(mSLog, "Initial switch status: {:s}",
                         mCurrentSwitchStatus.to_string());

      for (auto sys : mSwitchedMatrices) {
        SPDLOG_LOGGER_INFO(mSLog, "Switching System matrix {:s} \n{:s}",
                           sys.first.to_string(),
                           Logger::matrixToString(sys.second[0]));
      }
    }
    SPDLOG_LOGGER_INFO(mSLog, "Right side vector: \n{}", mRightSideVector);
  }
}

template <typename VarType> void MnaSolverMerged<VarType>::logLUTimes() {
  logFactorizationTime();
  logRecomputationTime();
  logSolveTime();
}

template <typename VarType> void MnaSolverMerged<VarType>::logSolveTime() {
  Real solveSum = 0.0;
  Real solveMax = 0.0;
  for (auto meas : mSolveTimes) {
    solveSum += meas;
    if (meas > solveMax)
      solveMax = meas;
  }
  SPDLOG_LOGGER_INFO(mSLog, "Cumulative solve times: {:.12f}", solveSum);
  SPDLOG_LOGGER_INFO(mSLog, "Average solve time: {:.12f}",
                     solveSum / static_cast<double>(mSolveTimes.size()));
  SPDLOG_LOGGER_INFO(mSLog, "Maximum solve time: {:.12f}", solveMax);
  SPDLOG_LOGGER_INFO(mSLog, "Number of solves: {:d}", mSolveTimes.size());
}

template <typename VarType>
void MnaSolverMerged<VarType>::logFactorizationTime() {
  for (auto meas : mFactorizeTimes) {
    SPDLOG_LOGGER_INFO(mSLog, "LU factorization time: {:.12f}", meas);
  }
}

template <typename VarType>
void MnaSolverMerged<VarType>::logRecomputationTime() {
  Real recompSum = 0.0;
  Real recompMax = 0.0;
  for (auto meas : mRecomputationTimes) {
    recompSum += meas;
    if (meas > recompMax)
      recompMax = meas;
  }
  // Sometimes, refactorization is not used
  if (mRecomputationTimes.size() != 0) {
    SPDLOG_LOGGER_INFO(mSLog, "Cumulative refactorization times: {:.12f}",
                       recompSum);
    SPDLOG_LOGGER_INFO(mSLog, "Average refactorization time: {:.12f}",
                       recompSum / ((double)mRecomputationTimes.size()));
    SPDLOG_LOGGER_INFO(mSLog, "Maximum refactorization time: {:.12f}",
                       recompMax);
    SPDLOG_LOGGER_INFO(mSLog, "Number of refactorizations: {:d}",
                       mRecomputationTimes.size());
  }
}

template <typename VarType>
std::shared_ptr<DirectLinearSolver>
MnaSolverMerged<VarType>::createDirectSolverImplementation(
    CPS::Logger::Log mSLog) {
  switch (this->mImplementationInUse) {
  case DirectLinearSolverImpl::DenseLU:
    return std::make_shared<DenseLUAdapter>(mSLog);
  case DirectLinearSolverImpl::SparseLU:
    return std::make_shared<SparseLUAdapter>(mSLog);
#ifdef WITH_KLU
  case DirectLinearSolverImpl::KLU:
    return std::make_shared<KLUAdapter>(mSLog);
#endif
#ifdef WITH_CUDA
  case DirectLinearSolverImpl::CUDADense:
    return std::make_shared<GpuDenseAdapter>(mSLog);
#ifdef WITH_CUDA_SPARSE
  case DirectLinearSolverImpl::CUDASparse:
    return std::make_shared<GpuSparseAdapter>(mSLog);
#endif
#ifdef WITH_MAGMA
  case DirectLinearSolverImpl::CUDAMagma:
    return std::make_shared<GpuMagmaAdapter>(mSLog);
#endif
#endif
  default:
    throw CPS::SystemError("unsupported linear solver implementation.");
  }
}

template <typename VarType>
void MnaSolverMerged<VarType>::setDirectLinearSolverImplementation(
    DirectLinearSolverImpl implementation) {
  this->mImplementationInUse = implementation;
}

template <typename VarType>
void MnaSolverMerged<VarType>::setDirectLinearSolverConfiguration(
    DirectLinearSolverConfiguration &configuration) {
  this->mConfigurationInUse = configuration;
}

} // namespace DPsim

template class DPsim::MnaSolverMerged<Real>;
template class DPsim::MnaSolverMerged<Complex>;
