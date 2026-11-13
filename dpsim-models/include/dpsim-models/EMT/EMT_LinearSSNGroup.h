// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <dpsim-models/EMT/EMT_LinearSSNModel.h>
#include <dpsim-models/MNASimPowerComp.h>

#include <chrono>
#include <unordered_map>

namespace CPS::EMT {

/// Paper-style SSN group generated from a collection of linear companion
/// models. Internal network nodes are eliminated once by a Schur complement;
/// only the retained terminal nodes remain in the surrounding MNA problem.
class LinearSSNGroup final : public MNASimPowerComp<Real> {
public:
  using Ptr = std::shared_ptr<LinearSSNGroup>;
  using Component = MNASimPowerComp<Real>;
  using ComponentList = std::vector<Component::Ptr>;

  /// Accelerator-neutral snapshot of the fully eliminated linear group.
  /// The matrices are the already-discrete maps used directly by the EMT
  /// timestep: h = historyMap*x and x+ = stateMap*x + inputMap*u.  It lets a
  /// device executor upload the realization once and keep its state resident.
  struct PrecomputedStateMaps {
    SparseMatrixRow historyMap;
    SparseMatrixRow stateMap;
    SparseMatrixRow inputMap;
    Matrix terminalAdmittance;
    Matrix initialState;
    std::vector<UInt> terminalMatrixIndices;
  };

  static Ptr make(String name, ComponentList components, SimNode::List allNodes,
                  SimNode::List retainedNodes,
                  Logger::Level level = Logger::Level::off) {
    return std::make_shared<LinearSSNGroup>(
        std::move(name), std::move(components), std::move(allNodes),
        std::move(retainedNodes), level);
  }

  LinearSSNGroup(String name, ComponentList components, SimNode::List allNodes,
                 SimNode::List retainedNodes,
                 Logger::Level level = Logger::Level::off);

  void initializeFromNodesAndTerminals(Real frequency) override;
  Bool mnaHasCompactRightSideVectorStamp() const override { return true; }
  void mnaAddCompactRightSideVectorStamp(Matrix &rightVector) override;
  void mnaWriteProfile(std::ostream &profile) const override;
  void setPrecomputedInternalMaps(Bool enabled) {
    mUsePrecomputedInternalMaps = enabled;
  }
  void setFullyReducedInternalMaps(Bool enabled) {
    mUseFullyReducedInternalMaps = enabled;
  }
  PrecomputedStateMaps precomputedStateMaps() const;

protected:
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompUpdateVoltage(const Matrix &) override {}
  void mnaCompUpdateCurrent(const Matrix &) override {}
  void mnaCompPreStep(Real, Int) override;
  void mnaCompPostStep(Real, Int, Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

private:
  ComponentList mComponents;
  SimNode::List mAllNodes;
  std::vector<String> mRetainedNames;
  std::vector<UInt> mRetainedMatrixIndices;
  std::vector<LinearSSNModel> mModels;
  Attribute<Matrix>::Ptr mState;
  SparseMatrixRow mF, mBi, mBn, mGin;
  // History maps are applied to dense vectors every step; contiguous rows
  // avoid the scattered output updates of column-major sparse storage.
  SparseMatrixRow mHi, mHn, mGni;
  Matrix mDd;
  Eigen::PartialPivLU<Matrix> mInternalFactor;
  Matrix mInternalHistoryInverse;
  // P = solve(Gii, Hi) and Q = solve(Gii, Gin). P*x is shared by the
  // terminal-history and post-step state-update paths; Q is also used when
  // forming the reduced terminal admittance.
  SparseMatrixRow mInternalStateMap;
  Matrix mInternalInputMap;
  SparseMatrixRow mInternalInputMapSparse;
  SparseMatrixRow mReducedHistoryMap;
  SparseMatrixRow mReducedStateMap;
  SparseMatrixRow mReducedInputMap;
  Bool mUseCachedInternalHistoryInverse = false;
  Bool mUsePrecomputedInternalMaps = true;
  Bool mUseFullyReducedInternalMaps = true;
  mutable Bool mInternalStateResponseReady = false;
  std::vector<UInt> mStateOffsets;
  std::vector<std::vector<UInt>> mModelInputIndices;
  std::unordered_map<String, UInt> mLocalNodeIndex;
  std::unordered_map<String, UInt> mRetainedNodeIndex;
  std::vector<UInt> mInternalIndices;
  std::vector<UInt> mTerminalIndices;
  // Reused per-step workspaces.  The paper realization is assembled once,
  // but its state/history vectors are updated at every EMT step.
  mutable Matrix mFullHistoryScratch;
  mutable Matrix mInternalHistoryScratch;
  mutable Matrix mInternalVoltageScratch;
  Matrix mLocalVoltageScratch;
  Matrix mTerminalInputScratch;
  mutable Matrix mTerminalHistoryScratch;
  Matrix mStateUpdateScratch;
  Bool mReady = false;
  mutable std::chrono::nanoseconds mHistoryMultiplyTime{};
  mutable std::chrono::nanoseconds mInternalSolveTime{};
  mutable std::chrono::nanoseconds mReducedHistoryTime{};
  mutable std::chrono::nanoseconds mRhsScatterTime{};
  mutable std::chrono::nanoseconds mStateUpdateTime{};
  mutable std::size_t mHistoryCalls = 0;
  mutable std::size_t mRhsScatterCalls = 0;
  mutable std::size_t mStateUpdateCalls = 0;

  void buildRealization(Real omega, Real timeStep,
                        Attribute<Matrix>::Ptr leftVector);
  void assembleFullHistory(Matrix &fullHistory) const;
  void currentHistory(Matrix &history) const;
  void terminalVoltage(const Matrix &leftVector, Matrix &voltage);
};

} // namespace CPS::EMT
