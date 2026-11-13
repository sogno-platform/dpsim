/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverDirect.h>

#include <dpsim-models/MathUtils.h>
#include <dpsim/SequentialScheduler.h>
#ifdef WITH_HIP
#include <dpsim/HipSparseBiCGStabAdapter.h>
#include <dpsim/HipSparseQrAdapter.h>
#endif

#include <algorithm>
#include <numeric>
#include <unordered_set>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverDirect<VarType>::MnaSolverDirect(String name, CPS::Domain domain,
                                          CPS::Logger::Level logLevel)
    : MnaSolver<VarType>(name, domain, logLevel) {
#ifdef WITH_KLU
  mImplementationInUse = DirectLinearSolverImpl::KLU;
#elif defined(WITH_SPARSE)
  mImplementationInUse = DirectLinearSolverImpl::SparseLU;
#else
  mImplementationInUse = DirectLinearSolverImpl::DenseLU;
#endif
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixEmpty(std::size_t index) {
  mSwitchedMatrices[std::bitset<SWITCH_NUM>(index)][0].setZero();
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixEmpty(std::size_t swIdx,
                                                   Int freqIdx) {
  mSwitchedMatrices[std::bitset<SWITCH_NUM>(swIdx)][freqIdx].setZero();
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixStamp(
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
void MnaSolverDirect<VarType>::switchedMatrixStamp(
    std::size_t swIdx, Int freqIdx, CPS::MNAInterface::List &components,
    CPS::MNASwitchInterface::List &switches) {
  const auto bit = std::bitset<SWITCH_NUM>(swIdx);
  auto &systemMatrix = mSwitchedMatrices[bit][freqIdx];
  for (auto component : components)
    component->mnaApplySystemMatrixStampHarm(systemMatrix, freqIdx);
  systemMatrix.makeCompressed();
  for (UInt i = 0; i < switches.size(); ++i)
    switches[i]->mnaApplySwitchSystemMatrixStamp(bit[i], systemMatrix, freqIdx);

  auto &directSolver = mDirectLinearSolvers[bit][freqIdx];
  directSolver->preprocessing(systemMatrix, mListVariableSystemMatrixEntries);
  const auto start = std::chrono::steady_clock::now();
  directSolver->factorize(systemMatrix);
  const auto end = std::chrono::steady_clock::now();
  mFactorizeTimes.push_back(std::chrono::duration<Real>(end - start).count());
}

template <typename VarType>
void MnaSolverDirect<VarType>::stampVariableSystemMatrix() {

  this->mDirectLinearSolverVariableSystemMatrix =
      createDirectSolverImplementation(mSLog);
  // TODO: a direct linear solver configuration is only applied if system matrix recomputation is used
  this->mDirectLinearSolverVariableSystemMatrix->setConfiguration(
      mConfigurationInUse);
  if (auto dense = std::dynamic_pointer_cast<DenseLUAdapter>(
          mDirectLinearSolverVariableSystemMatrix))
    dense->setDetailedProfiling(Solver::mLogSolveTimes);

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

  // Now stamp initial state of variable elements and switches into matrix
  SPDLOG_LOGGER_INFO(mSLog, "Stamping variable elements");
  {
    // Preserve every potentially changing coefficient in the sparse pattern,
    // including entries that are zero in the initial operating point. The
    // pattern must remain fixed when the value cache is used during EMT steps.
    CPS::Math::ScopedZeroSkipping zeroSkippingScope(false);
    for (auto varElem : mMNAIntfVariableComps)
      varElem->mnaApplySystemMatrixStamp(mVariableSystemMatrix);
  }

  // Keep structural slots for entries that can change later, even if their
  // initial stamp is zero and zero skipping is enabled.
  for (const auto &entry : mListVariableSystemMatrixEntries)
    mVariableSystemMatrix.coeffRef(entry.first, entry.second) += 0.;

  // Keep the reset matrix on exactly the same sparse pattern and in the same
  // coefficient order as the variable matrix. Variable-only entries must be
  // retained as explicit zeros here: recomputeSystemMatrix() copies values by
  // array position, and the static-only pattern can have a different ordering
  // once variable stamps have inserted additional entries.
  mBaseSystemMatrix = mVariableSystemMatrix;
  std::fill_n(mBaseSystemMatrix.valuePtr(), mBaseSystemMatrix.nonZeros(), 0.);
  for (auto statElem : mMNAComponents)
    statElem->mnaApplySystemMatrixStamp(mBaseSystemMatrix);

  SPDLOG_LOGGER_INFO(mSLog, "Initial system matrix with variable elements {}",
                     Logger::matrixToString(mVariableSystemMatrix));
  /* TODO: find replacement for flush() */
  mSLog->flush();

  // Calculate factorization of current matrix
  mDirectLinearSolverVariableSystemMatrix->preprocessing(
      mVariableSystemMatrix, mListVariableSystemMatrixEntries);
  CPS::Math::prepareSparseMatrixValueCache(mVariableSystemMatrix);

  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolverVariableSystemMatrix->factorize(mVariableSystemMatrix);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mFactorizeTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverDirect<VarType>::solveWithSystemMatrixRecomputation(
    Real time, Int timeStepCount) {
  auto profileStart = std::chrono::steady_clock::now();
  // Reset source vector
#ifdef WITH_HIP
  if (!mHipDeviceAssemblyReady)
#endif
    mRightSideVector.setZero();
  auto profileEnd = std::chrono::steady_clock::now();
  if (Solver::mLogSolveTimes)
    recordProfileMeasurement("rhs_zero", profileStart, profileEnd);

#ifdef WITH_HIP
  updateHipSG4PreStep(time);
  updateHipLinearSSNHistories();
#endif

  // Add together the right side vector (computed by the components'
  // pre-step tasks)
  profileStart = std::chrono::steady_clock::now();
#ifdef WITH_HIP
  if (!mHipDeviceAssemblyReady)
#endif
    addRightVectorStamps(mRightSideVector);
  profileEnd = std::chrono::steady_clock::now();
  if (Solver::mLogSolveTimes)
    recordProfileMeasurement("rhs_assembly", profileStart, profileEnd);

  // Get switch and variable comp status and update system matrix and lu factorization accordingly
  profileStart = std::chrono::steady_clock::now();
#ifdef WITH_HIP
  if (mHipDeviceAssemblyReady)
    mVariableComponentChanged = false;
  else
    mVariableComponentChanged = hasVariableComponentChanged();
#else
  mVariableComponentChanged = hasVariableComponentChanged();
#endif
  profileEnd = std::chrono::steady_clock::now();
  if (Solver::mLogSolveTimes)
    recordProfileMeasurement("variable_change_detection", profileStart,
                             profileEnd);
  if (mVariableComponentChanged)
    recomputeSystemMatrix(time);
#ifdef WITH_HIP
  if (mHipSparseDeviceSolver)
    for (auto &adapter : mHipSG4Adapters)
      adapter->scatterDeviceMatrix();
  if (mHipDeviceAssemblyReady && mHipSparseDeviceSolver)
    mHipSparseDeviceSolver->refactorizeDevice();
#endif

  // Calculate new solution vector
  auto start = std::chrono::steady_clock::now();
  bool synchronizeNodeOutputs = true;
#ifdef WITH_HIP
  synchronizeNodeOutputs =
      !mHipDeviceAssemblyReady || mSLog->should_log(spdlog::level::info);
  if (mHipSparseDeviceSolver) {
    if (mHipDeviceAssemblyReady)
      mHipSparseDeviceSolver->clearDeviceRightHandSide();
    else
      mHipSparseDeviceSolver->uploadRightHandSide(mRightSideVector);
    for (auto &adapter : mHipSG4Adapters)
      adapter->accumulateDeviceRhs();
    for (auto &adapter : mHipLinearSSNAdapters)
      adapter->accumulateDeviceRhs();
    mHipSparseDeviceSolver->solveDevice(synchronizeNodeOutputs);
    if (synchronizeNodeOutputs)
      mHipSparseDeviceSolver->copyDeviceSolutionToHost(*mLeftSideVectorValue);
  } else
#endif
  {
    mDirectLinearSolverVariableSystemMatrix->solveInto(mRightSideVector,
                                                       *mLeftSideVectorValue);
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mSolveTimes.push_back(diff.count());
  if (Solver::mLogSolveTimes)
    recordProfileMeasurement("numeric_solve", start, end);

#ifdef WITH_HIP
  if (mHipSparseDeviceSolver) {
    for (auto &adapter : mHipSG4Adapters)
      adapter->postStepDevice(*mLeftSideVectorValue,
                              mHipSparseDeviceSolver->deviceSolution(),
                              synchronizeNodeOutputs);
    if (mHipDeviceAssemblyReady)
      for (auto &adapter : mHipLinearSSNAdapters)
        adapter->updateStatesDevice(*mLeftSideVectorValue,
                                    mHipSparseDeviceSolver->deviceSolution());
    else
      updateHipLinearSSNStates(*mLeftSideVectorValue);
  } else {
    updateHipSG4PostStep(*mLeftSideVectorValue);
    updateHipLinearSSNStates(*mLeftSideVectorValue);
  }
#endif

  // The linear solve above uses the tangent prepared by component pre-steps.
  // Repeat only for opt-in nonlinear components; existing variable SSN models
  // keep their previous single-linearization behavior.
  if (!mNonlinearComps.empty()) {
    constexpr Int maxNewtonIterations = 40;
    bool converged = false;
    for (Int iteration = 0; iteration < maxNewtonIterations; ++iteration) {
      bool allConverged = true;
      for (auto &component : mNonlinearComps)
        allConverged =
            component->mnaCheckNonlinearConvergence(**mLeftSideVector) &&
            allConverged;
      if (allConverged) {
        converged = true;
        break;
      }
      for (auto &component : mNonlinearComps)
        component->mnaPrepareNonlinearIteration();
      recomputeSystemMatrix(time);
      mVariableComponentChanged = true;
      mRightSideVector.setZero();
      for (auto stamp : mRightVectorStamps)
        mRightSideVector += *stamp;
      **mLeftSideVector =
          mDirectLinearSolverVariableSystemMatrix->solve(mRightSideVector);
    }
    if (!converged)
      throw CPS::SystemError(
          "Nonlinear MNA Newton iteration failed to converge at time " +
          std::to_string(time));
  }

  // TODO split into separate task? (dependent on x, updating all v attributes)
  profileStart = std::chrono::steady_clock::now();
  if (synchronizeNodeOutputs)
    for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
      mNodes[nodeIdx]->mnaUpdateVoltage(*mLeftSideVectorValue);
  profileEnd = std::chrono::steady_clock::now();
  if (Solver::mLogSolveTimes)
    recordProfileMeasurement("node_voltage_propagation", profileStart,
                             profileEnd);

  // Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverDirect<VarType>::recomputeSystemMatrix(Real time) {
  CPS::Math::ScopedZeroSkipping zeroSkippingScope(this->mZeroSkipping);
  const auto profileStart = std::chrono::steady_clock::now();
  // The sparse structure is fixed after initialization. Reset only the
  // coefficient values instead of copying/reallocating the full sparse
  // matrix on every timestep.
  std::copy_n(mBaseSystemMatrix.valuePtr(), mBaseSystemMatrix.nonZeros(),
              mVariableSystemMatrix.valuePtr());

  // Now stamp variable elements and switches into matrix
  for (auto comp : mMNAIntfVariableComps)
    comp->mnaApplySystemMatrixStamp(mVariableSystemMatrix);
  const auto profileEnd = std::chrono::steady_clock::now();
  if (Solver::mLogSolveTimes)
    recordProfileMeasurement("matrix_reset_and_variable_stamping", profileStart,
                             profileEnd);

  // Refactorization of matrix assuming that structure remained
  // constant by omitting analyzePattern
  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolverVariableSystemMatrix->partialRefactorize(
      mVariableSystemMatrix, mListVariableSystemMatrixEntries);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mRecomputationTimes.push_back(diff.count());
  if (Solver::mLogSolveTimes)
    recordProfileMeasurement("partial_refactorization", start, end);
  ++mNumRecomputations;
}

template <typename VarType>
void MnaSolverDirect<VarType>::extractStateSpace(Real time) {
  if (!mStateSpaceExtractor)
    return;

  if (mSystemMatrixRecomputationEnabled) {
    mStateSpaceExtractor->extract(*mDirectLinearSolverVariableSystemMatrix,
                                  mVariableComponentChanged,
                                  mVariableComponentChanged, time);
  } else {
    mStateSpaceExtractor->extract(
        *mDirectLinearSolvers[mCurrentSwitchStatus][0], false,
        mSystemMatrixChanged, time);
  }
}

template <> void MnaSolverDirect<Real>::createEmptySystemMatrix() {
  if (mSwitches.size() > SWITCH_NUM)
    throw SystemError("Too many Switches.");

  if (mSystemMatrixRecomputationEnabled) {
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

template <> void MnaSolverDirect<Complex>::createEmptySystemMatrix() {
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
  } else if (mSystemMatrixRecomputationEnabled) {
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
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createSolveTask() {
  prepareCompactRightVectorStamps();
#ifdef WITH_HIP
  initializeHipLinearSSNAdapters();
#endif
  return std::make_shared<MnaSolverDirect<VarType>::SolveTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createSolveTaskRecomp() {
  prepareCompactRightVectorStamps();
#ifdef WITH_HIP
  initializeHipLinearSSNAdapters();
#endif
  return std::make_shared<MnaSolverDirect<VarType>::SolveTaskRecomp>(*this);
}

#ifdef WITH_HIP
template <typename VarType>
void MnaSolverDirect<VarType>::initializeHipLinearSSNAdapters() {
  if (mHipAdaptersInitialized)
    return;
  if (mImplementationInUse != DirectLinearSolverImpl::HIPDense &&
      mImplementationInUse != DirectLinearSolverImpl::HIPSparseBiCGStab &&
      mImplementationInUse != DirectLinearSolverImpl::HIPSparseLURefactor &&
      mImplementationInUse != DirectLinearSolverImpl::HIPSparseQR)
    return;
  mHipAdaptersInitialized = true;
  std::vector<std::shared_ptr<CPS::EMT::Ph3::SynchronGenerator4OrderVBR>>
      supportedGenerators;
  std::unordered_set<const CPS::MNAInterface *> seen;
  auto initializeComponent = [this, &supportedGenerators,
                              &seen](const auto &component) {
    if (!component || !seen.insert(component.get()).second)
      return;
    if (auto fused =
            std::dynamic_pointer_cast<CPS::EMT::LinearSSNFused>(component))
      mHipLinearSSNAdapters.emplace_back(
          std::make_unique<HipLinearSSNFusedAdapter>(*fused));
    if (auto generator = std::dynamic_pointer_cast<
            CPS::EMT::Ph3::SynchronGenerator4OrderVBR>(component))
      if (generator->supportsHipBatch())
        supportedGenerators.push_back(std::move(generator));
  };
  for (const auto &component : mMNAComponents)
    initializeComponent(component);
  for (const auto &component : mMNAIntfVariableComps)
    initializeComponent(component);
  if (!supportedGenerators.empty())
    mHipSG4Adapters.emplace_back(
        std::make_unique<HipSG4BatchAdapter>(std::move(supportedGenerators)));
  if ((mImplementationInUse == DirectLinearSolverImpl::HIPSparseBiCGStab ||
       mImplementationInUse == DirectLinearSolverImpl::HIPSparseLURefactor) &&
      !mHipSG4Adapters.empty()) {
    bool allRightStampsManaged = !mRightVectorStampOperations.empty();
    for (const auto &operation : mRightVectorStampOperations) {
      if (!operation.compact) {
        allRightStampsManaged = false;
        break;
      }
      auto *generator =
          dynamic_cast<CPS::EMT::Ph3::SynchronGenerator4OrderVBR *>(
              operation.compact);
      const bool isSg4 = generator && generator->supportsHipBatch();
      const bool isFused =
          dynamic_cast<CPS::EMT::LinearSSNFused *>(operation.compact);
      if (!isSg4 && !isFused) {
        allRightStampsManaged = false;
        break;
      }
    }
    if (!allRightStampsManaged)
      throw std::runtime_error(
          "HIP sparse solvers require fully supported device-side RHS "
          "assembly; select the IEEE118 allssn aggregate configuration.");
    mHipSparseDeviceSolver = dynamic_cast<HipSparseDeviceSolver *>(
        mDirectLinearSolverVariableSystemMatrix.get());
    if (!mHipSparseDeviceSolver)
      throw std::runtime_error(
          "HIP sparse implementation did not create its device solver");
    for (auto &adapter : mHipSG4Adapters) {
      adapter->configureDeviceAssembly(
          mVariableSystemMatrix, mBaseSystemMatrix,
          mHipSparseDeviceSolver->deviceValues(),
          mHipSparseDeviceSolver->deviceRightHandSide());
    }
    for (auto &adapter : mHipLinearSSNAdapters)
      adapter->configureDeviceRhs(
          mHipSparseDeviceSolver->deviceRightHandSide());
    mHipDeviceAssemblyReady = true;
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::updateHipLinearSSNHistories() {
  for (auto &adapter : mHipLinearSSNAdapters)
    adapter->updateHistories();
}

template <typename VarType>
void MnaSolverDirect<VarType>::updateHipLinearSSNStates(
    const Matrix &leftVector) {
  for (auto &adapter : mHipLinearSSNAdapters)
    adapter->updateStates(leftVector);
}

template <typename VarType>
void MnaSolverDirect<VarType>::updateHipSG4PreStep(Real time) {
  for (auto &adapter : mHipSG4Adapters)
    adapter->preStep(time);
}

template <typename VarType>
void MnaSolverDirect<VarType>::updateHipSG4PostStep(const Matrix &leftVector) {
  for (auto &adapter : mHipSG4Adapters)
    adapter->postStep(leftVector);
}
#endif

template <typename VarType>
void MnaSolverDirect<VarType>::prepareCompactRightVectorStamps() {
  std::unordered_map<const Matrix *, CPS::MNAInterface *> compactStamps;
  auto collect = [&compactStamps](const auto &components) {
    for (const auto &component : components) {
      if (!component->mnaHasCompactRightSideVectorStamp())
        continue;
      auto stamp = component->getRightVector();
      if (stamp->get().size() != 0)
        compactStamps.emplace(&stamp->get(), component.get());
    }
  };
  collect(mMNAComponents);
  collect(mMNAIntfVariableComps);

  // Resolve compact-vs-dense accumulation once. The solve loop then follows
  // this ordered list directly instead of hashing every RHS stamp each step.
  mRightVectorStampOperations.clear();
  mRightVectorStampOperations.reserve(mRightVectorStamps.size());
  for (const Matrix *stamp : mRightVectorStamps) {
    const auto compact = compactStamps.find(stamp);
    if (compact != compactStamps.end())
      mRightVectorStampOperations.push_back({nullptr, compact->second});
    else
      mRightVectorStampOperations.push_back({stamp, nullptr});
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::addRightVectorStamps(Matrix &rightVector) {
  using Clock = std::chrono::steady_clock;
  const bool profile = Solver::mLogSolveTimes;
  const auto start = profile ? Clock::now() : Clock::time_point{};
  std::chrono::nanoseconds compactTime{};
  for (const auto &operation : mRightVectorStampOperations) {
    if (operation.compact) {
      const auto compactStart = profile ? Clock::now() : Clock::time_point{};
      operation.compact->mnaAddCompactRightSideVectorStamp(rightVector);
      if (profile)
        compactTime += std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - compactStart);
      ++mCompactRightVectorStampCalls;
    } else {
      rightVector += *operation.dense;
      ++mDenseRightVectorStampCalls;
    }
  }
  if (profile) {
    const auto end = Clock::now();
    recordProfileMeasurement("rhs_compact_stamp_callbacks", start,
                             start + compactTime);
    recordProfileMeasurement("rhs_dense_stamp_accumulation",
                             start + compactTime, end);
  }
}

template <typename VarType>
std::shared_ptr<CPS::Task>
MnaSolverDirect<VarType>::createSolveTaskHarm(UInt freqIdx) {
  return std::make_shared<MnaSolverDirect<VarType>::SolveTaskHarm>(*this,
                                                                   freqIdx);
}

template <typename VarType>
std::shared_ptr<CPS::Task>
MnaSolverDirect<VarType>::createStateSpaceExtractionTask() {
  return std::make_shared<
      typename MnaSolverDirect<VarType>::StateSpaceExtractionTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createLogTask() {
  return std::make_shared<MnaSolverDirect<VarType>::LogTask>(*this);
}

template <typename VarType>
void MnaSolverDirect<VarType>::solve(Real time, Int timeStepCount) {
  const bool profile = Solver::mLogSolveTimes;
  const auto previousSwitchStatus = mCurrentSwitchStatus;
  // Reset source vector
  auto profileStart = profile ? std::chrono::steady_clock::now()
                              : std::chrono::steady_clock::time_point{};
  mRightSideVector.setZero();
  if (profile)
    recordProfileMeasurement("rhs_zero", profileStart,
                             std::chrono::steady_clock::now());

#ifdef WITH_HIP
  updateHipSG4PreStep(time);
  updateHipLinearSSNHistories();
#endif

  // Add together the right side vector (computed by the components' pre-step tasks)
  addRightVectorStamps(mRightSideVector);

  if (!mIsInInitialization) {
    profileStart = profile ? std::chrono::steady_clock::now()
                           : std::chrono::steady_clock::time_point{};
    MnaSolver<VarType>::updateSwitchStatus();
    if (profile)
      recordProfileMeasurement("switch_status_update", profileStart,
                               std::chrono::steady_clock::now());
  }

  if (mSwitchedMatrices.size() > 0) {
    std::chrono::steady_clock::time_point start;
    if (profile)
      start = std::chrono::steady_clock::now();

    mDirectLinearSolvers[mCurrentSwitchStatus][0]->solveInto(
        mRightSideVector, *mLeftSideVectorValue);

    if (profile) {
      auto end = std::chrono::steady_clock::now();
      std::chrono::duration<Real> diff = end - start;
      mSolveTimes.push_back(diff.count());
      recordProfileMeasurement("numeric_solve", start, end);
    }
  }

  // CHECK: Is this really required? Or can operations actually become part of
  // correctorStep and mnaPostStep?
  profileStart = profile ? std::chrono::steady_clock::now()
                         : std::chrono::steady_clock::time_point{};
  for (auto syncGen : mSyncGen)
    syncGen->updateVoltage(*mLeftSideVectorValue);
  if (profile)
    recordProfileMeasurement("iterative_generator_voltage_update", profileStart,
                             std::chrono::steady_clock::now());

  // Reset number of iterations
  mIter = 0;

  // Additional solve steps for iterative models
  if (mSyncGen.size() > 0) {
    UInt numCompsRequireIter;
    do {
      // count synchronous generators that require iteration
      numCompsRequireIter = 0;
      profileStart = profile ? std::chrono::steady_clock::now()
                             : std::chrono::steady_clock::time_point{};
      for (auto syncGen : mSyncGen)
        if (syncGen->requiresIteration())
          numCompsRequireIter++;
      if (profile)
        recordProfileMeasurement("iterative_generator_convergence_check",
                                 profileStart,
                                 std::chrono::steady_clock::now());

      // recompute solve step if at least one component demands iteration
      if (numCompsRequireIter > 0) {
        mIter++;

        // Reset source vector
        mRightSideVector.setZero();

        if (!mIsInInitialization)
          MnaSolver<VarType>::updateSwitchStatus();

        profileStart = profile ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
        for (auto syncGen : mSyncGen)
          syncGen->correctorStep();
        if (profile)
          recordProfileMeasurement("iterative_generator_corrector",
                                   profileStart,
                                   std::chrono::steady_clock::now());

        // Add together the right side vector (computed by the components' pre-step tasks)
        addRightVectorStamps(mRightSideVector);

        if (mSwitchedMatrices.size() > 0) {
          auto start = std::chrono::steady_clock::now();
          mDirectLinearSolvers[mCurrentSwitchStatus][0]->solveInto(
              mRightSideVector, *mLeftSideVectorValue);
          auto end = std::chrono::steady_clock::now();
          std::chrono::duration<Real> diff = end - start;
          mSolveTimes.push_back(diff.count());
          if (profile)
            recordProfileMeasurement("numeric_solve", start, end);
        }

        // CHECK: Is this really required? Or can operations actually become part of
        // correctorStep and mnaPostStep?
        profileStart = profile ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
        for (auto syncGen : mSyncGen)
          syncGen->updateVoltage(*mLeftSideVectorValue);
        if (profile)
          recordProfileMeasurement("iterative_generator_voltage_update",
                                   profileStart,
                                   std::chrono::steady_clock::now());
      }
    } while (numCompsRequireIter > 0);
  }

#ifdef WITH_HIP
  updateHipSG4PostStep(*mLeftSideVectorValue);
  updateHipLinearSSNStates(*mLeftSideVectorValue);
#endif

  mSystemMatrixChanged = (mCurrentSwitchStatus != previousSwitchStatus);

  // TODO split into separate task? (dependent on x, updating all v attributes)
  profileStart = profile ? std::chrono::steady_clock::now()
                         : std::chrono::steady_clock::time_point{};
  for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
    mNodes[nodeIdx]->mnaUpdateVoltage(*mLeftSideVectorValue);
  if (profile)
    recordProfileMeasurement("node_voltage_propagation", profileStart,
                             std::chrono::steady_clock::now());

  // Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverDirect<VarType>::solveWithHarmonics(Real time, Int timeStepCount,
                                                  Int freqIdx) {
  mRightSideVectorHarm[freqIdx].setZero();

  // Sum of right side vectors (computed by the components' pre-step tasks)
  for (auto stamp : mRightVectorStamps)
    mRightSideVectorHarm[freqIdx] += stamp->col(freqIdx);

  // Frequency solve tasks execute concurrently. Derive the switch-state key
  // locally instead of mutating the shared mCurrentSwitchStatus bitset from
  // each frequency worker.
  std::bitset<SWITCH_NUM> switchStatus;
  for (UInt i = 0; i < mSwitches.size(); ++i)
    switchStatus.set(i, mSwitches[i]->mnaIsClosed());

  mDirectLinearSolvers[switchStatus][freqIdx]->solveInto(
      mRightSideVectorHarm[freqIdx], **mLeftSideVectorHarm[freqIdx]);
}

template <typename VarType> void MnaSolverDirect<VarType>::logSystemMatrices() {
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

  } else if (mSystemMatrixRecomputationEnabled) {
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

template <typename VarType> void MnaSolverDirect<VarType>::logLUTimes() {
  logFactorizationTime();
  logRecomputationTime();
  logSolveTime();
  for (const auto &entry : mProfileMeasurements) {
    const auto &measurement = entry.second;
    const double averageNs =
        measurement.count == 0
            ? 0.
            : static_cast<double>(measurement.total.count()) /
                  measurement.count;
    SPDLOG_LOGGER_INFO(mSLog,
                       "DEEP_TIMING_PHASE,{},{:.9f},{:.3f},{:.3f},{:.3f},{}",
                       entry.first, measurement.total.count() * 1e-9,
                       averageNs * 1e-3, measurement.minimum.count() * 1e-3,
                       measurement.maximum.count() * 1e-3, measurement.count);
  }
  std::ofstream phaseFile(CPS::Logger::logDir() + "/" + this->mName +
                          "_SolverPhaseTimes.csv");
  phaseFile << "phase,total_s,avg_us,min_us,max_us,calls\n";
  for (const auto &entry : mProfileMeasurements) {
    const auto &measurement = entry.second;
    const double count = static_cast<double>(measurement.count);
    phaseFile << entry.first << "," << measurement.total.count() * 1e-9 << ","
              << (count ? measurement.total.count() / count * 1e-3 : 0.) << ","
              << measurement.minimum.count() * 1e-3 << ","
              << measurement.maximum.count() * 1e-3 << "," << measurement.count
              << "\n";
  }
  phaseFile << "rhs_dense_stamp_component_calls,0,0,0,0,"
            << mDenseRightVectorStampCalls << "\n";
  phaseFile << "rhs_compact_stamp_component_calls,0,0,0,0,"
            << mCompactRightVectorStampCalls << "\n";
  for (const auto &operation : mRightVectorStampOperations)
    if (operation.compact)
      operation.compact->mnaWriteProfile(phaseFile);
  if (auto dense = std::dynamic_pointer_cast<DenseLUAdapter>(
          mDirectLinearSolverVariableSystemMatrix))
    dense->writeDetailedProfile(phaseFile);
  if (auto dense = std::dynamic_pointer_cast<DenseLUAdapter>(
          mDirectLinearSolverVariableSystemMatrix)) {
    std::ofstream diagnostics(CPS::Logger::logDir() + "/" + this->mName +
                              "_SolverDiagnostics.csv");
    dense->writeDetailedDiagnostics(diagnostics);
  }
  std::ofstream matrixFile(CPS::Logger::logDir() + "/" + this->mName +
                           "_SolverMatrixStats.csv");
  const SparseMatrix *profiledMatrix = &mVariableSystemMatrix;
  const char *matrixSource = "variable_system_matrix";
  if (!mSystemMatrixRecomputationEnabled) {
    const auto activeMatrix = mSwitchedMatrices.find(mCurrentSwitchStatus);
    if (activeMatrix != mSwitchedMatrices.end() &&
        !activeMatrix->second.empty()) {
      profiledMatrix = &activeMatrix->second.front();
      matrixSource = "active_switched_matrix";
    }
  }
  matrixFile
      << "source,rows,cols,nonzeros,structural_density,variable_entries\n";
  const double matrixSize = static_cast<double>(profiledMatrix->rows());
  const double matrixEntries = matrixSize * matrixSize;
  matrixFile << matrixSource << "," << profiledMatrix->rows() << ","
             << profiledMatrix->cols() << "," << profiledMatrix->nonZeros()
             << ","
             << (matrixEntries ? profiledMatrix->nonZeros() / matrixEntries
                               : 0.)
             << "," << mListVariableSystemMatrixEntries.size() << "\n";
  if (!mFactorizeTimes.empty()) {
    const Real total = std::accumulate(mFactorizeTimes.begin(),
                                       mFactorizeTimes.end(), Real(0.));
    const auto bounds =
        std::minmax_element(mFactorizeTimes.begin(), mFactorizeTimes.end());
    phaseFile << "initial_factorization," << total << ","
              << total * 1e6 / mFactorizeTimes.size() << ","
              << *bounds.first * 1e6 << "," << *bounds.second * 1e6 << ","
              << mFactorizeTimes.size() << "\n";
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::recordProfileMeasurement(
    const String &phase, std::chrono::steady_clock::time_point start,
    std::chrono::steady_clock::time_point end) {
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  auto &measurement = mProfileMeasurements[phase];
  if (measurement.count == 0) {
    measurement.minimum = elapsed;
    measurement.maximum = elapsed;
  } else {
    measurement.minimum = std::min(measurement.minimum, elapsed);
    measurement.maximum = std::max(measurement.maximum, elapsed);
  }
  measurement.total += elapsed;
  ++measurement.count;
}

template <typename VarType> void MnaSolverDirect<VarType>::logSolveTime() {
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
void MnaSolverDirect<VarType>::logFactorizationTime() {
  for (auto meas : mFactorizeTimes) {
    SPDLOG_LOGGER_INFO(mSLog, "LU factorization time: {:.12f}", meas);
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::logRecomputationTime() {
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
MnaSolverDirect<VarType>::createDirectSolverImplementation(
    CPS::Logger::Log mSLog) {
  switch (this->mImplementationInUse) {
  case DirectLinearSolverImpl::DenseLU:
    return std::make_shared<DenseLUAdapter>(mSLog);
#ifdef WITH_SPARSE
  case DirectLinearSolverImpl::SparseLU:
    return std::make_shared<SparseLUAdapter>(mSLog);
#endif // WITH_SPARSE
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
#ifdef WITH_METAL
  case DirectLinearSolverImpl::MetalDense:
    return std::make_shared<MetalDenseAdapter>(mSLog);
  case DirectLinearSolverImpl::MetalSparseBiCGSTAB:
    return std::make_shared<MetalSparseBiCGSTABAdapter>(mSLog);
#endif
#ifdef WITH_HIP
  case DirectLinearSolverImpl::HIPSparseQR:
    return std::make_shared<HipSparseQrAdapter>(mSLog);
  case DirectLinearSolverImpl::HIPDense:
    return std::make_shared<HipDenseAdapter>(mSLog);
  case DirectLinearSolverImpl::HIPSparseBiCGStab:
    return std::make_shared<HipSparseBiCGStabAdapter>(mSLog);
  case DirectLinearSolverImpl::HIPSparseLURefactor:
#ifdef WITH_KLU
    return std::make_shared<HipSparseLURefactorAdapter>(mSLog);
#else
    throw CPS::SystemError(
        "HIP sparse LU refactorization requires the KLU seed solver");
#endif
#endif
  default:
    throw CPS::SystemError("unsupported linear solver implementation.");
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::setDirectLinearSolverImplementation(
    DirectLinearSolverImpl implementation) {
  this->mImplementationInUse = implementation;
}

template <typename VarType>
void MnaSolverDirect<VarType>::setDirectLinearSolverConfiguration(
    DirectLinearSolverConfiguration &configuration) {
  this->mConfigurationInUse = configuration;
}

} // namespace DPsim

template class DPsim::MnaSolverDirect<Real>;
template class DPsim::MnaSolverDirect<Complex>;
