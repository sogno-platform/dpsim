// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <dpsim-models/EMT/EMT_LinearSSNModel.h>
#include <dpsim-models/MNASimPowerComp.h>

namespace CPS::EMT {

/// A mathematically fused linear SSN network.
///
/// Child SSN components are used only as initialization-time model exporters.
/// At run time this component owns one concatenated state collection, stamps
/// all exported port admittances directly, and evaluates all histories and
/// state updates in one flat execution plan.
class LinearSSNFused final : public MNASimPowerComp<Real> {
public:
  using Ptr = std::shared_ptr<LinearSSNFused>;
  using Component = MNASimPowerComp<Real>;
  using ComponentList = std::vector<Component::Ptr>;

  static Ptr make(String name, ComponentList components, SimNode::List nodes,
                  Logger::Level level = Logger::Level::off) {
    return std::make_shared<LinearSSNFused>(
        std::move(name), std::move(components), std::move(nodes), level);
  }

  LinearSSNFused(String name, ComponentList components, SimNode::List nodes,
                 Logger::Level level = Logger::Level::off);

  void initializeFromNodesAndTerminals(Real frequency) override;

  Bool mnaHasCompactRightSideVectorStamp() const override { return true; }
  void mnaAddCompactRightSideVectorStamp(Matrix &rightVector) override;

  /// Enable an external device executor for the fused component's runtime
  /// state. The device executor must keep the model states current and call
  /// setDeviceHistories() before the component's RHS stamp is requested.
  void setDeviceManaged(bool enabled) noexcept { mDeviceManaged = enabled; }
  void setDeviceRhsManaged(bool enabled) noexcept {
    mDeviceRhsManaged = enabled;
  }
  const std::vector<LinearSSNModel> &models() const noexcept { return mModels; }
  void setDeviceHistories(const std::vector<Matrix> &histories);

protected:
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
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
  std::vector<LinearSSNModel> mModels;
  std::vector<Matrix> mHistories;
  Attribute<Matrix>::Ptr mFusedState;
  Bool mModelsReady = false;
  Bool mDeviceManaged = false;
  Bool mDeviceRhsManaged = false;

  void updateHistories();
  void addHistoryStamp(Matrix &rightVector) const;
  void updateStates(const Matrix &leftVector);
  static Real vectorValue(const Matrix &vector, UInt index);
  static void addVectorValue(Matrix &vector, UInt index, Real value);
};

} // namespace CPS::EMT
