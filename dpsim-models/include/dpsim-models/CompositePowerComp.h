// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/MNASimPowerComp.h>

namespace CPS {
enum class MNA_SUBCOMP_TASK_ORDER {
  NO_TASK,
  TASK_BEFORE_PARENT,
  TASK_AFTER_PARENT
};

/// Base class for composite power components
template <typename VarType>
class CompositePowerComp : public MNASimPowerComp<VarType> {

private:
  MNAInterface::List mSubcomponentsMNA;
  MNAInterface::List mSubcomponentsPreStepBeforeParent;
  MNAInterface::List mSubcomponentsPreStepAfterParent;
  MNAInterface::List mSubcomponentsPostStepBeforeParent;
  MNAInterface::List mSubcomponentsPostStepAfterParent;

  std::vector<CPS::Attribute<Matrix>::Ptr> mRightVectorStamps;

protected:
  /// Guards createSubComponents() against double-execution; set it to true at
  /// the top of every override. Provided here so subclasses need not redeclare.
  bool mSubCompCreated = false;

public:
  using Type = VarType;
  using Ptr = std::shared_ptr<CompositePowerComp<VarType>>;
  using List = std::vector<Ptr>;

  /// Basic constructor that takes UID, name and log level
  CompositePowerComp(String uid, String name, Bool hasPreStep, Bool hasPostStep,
                     Logger::Level logLevel)
      : MNASimPowerComp<VarType>(uid, name, hasPreStep, hasPostStep, logLevel) {
  }

  /// Basic constructor that takes name and log level and sets the UID to name as well
  CompositePowerComp(String name, Bool hasPreStep = true,
                     Bool hasPostStep = true,
                     Logger::Level logLevel = Logger::Level::off)
      : CompositePowerComp<VarType>(name, name, hasPreStep, hasPostStep,
                                    logLevel) {}

  /// Destructor - does not do anything
  virtual ~CompositePowerComp() = default;

  /// Returns the MNA subcomponents owned by this composite component.
  ///
  /// The returned list remains owned by the composite and must not be modified
  /// by callers.
  const MNAInterface::List &mnaSubComponents() const noexcept {
    return mSubcomponentsMNA;
  }

  /// Sequences the three init stages: createSubComponents(), then
  /// initializeParentFromNodesAndTerminals(), then recurse into sub-components.
  /// See docs Overview/subcomponents.md.
  void initializeFromNodesAndTerminals(Real frequency) override final;

  /// Pure-virtual hook for the parameterization stage. Concrete composites
  /// implement this instead of overriding initializeFromNodesAndTerminals().
  /// Reads terminal data, computes frequency-dependent values, and calls
  /// setParameters() on sub-components. The simulation frequency is passed
  /// directly so there is no need to access mFrequencies(0,0).
  virtual void initializeParentFromNodesAndTerminals(Real frequency) = 0;

  /// @brief Add a new subcomponent implementing MNA methods
  /// @param subc The new subcomponent
  /// @param preStepOrder When to execute the subcomponent's pre-step in relation to the parent
  /// @param postStepOrder When to execute the subcomponent's post-step in relation to the parent
  void addMNASubComponent(typename SimPowerComp<VarType>::Ptr subc,
                          MNA_SUBCOMP_TASK_ORDER preStepOrder,
                          MNA_SUBCOMP_TASK_ORDER postStepOrder,
                          Bool contributeToRightVector);

  // #### MNA Interface Functions ####
  /// Initializes variables of components
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
  /// Initializes variables of components, harmonics domain
  void mnaCompInitializeHarm(
      Real omega, Real timeStep,
      std::vector<Attribute<Matrix>::Ptr> leftVectors) override;
  /// Stamps system matrix for a single frequency index
  void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                         Int freqIdx) override;
  /// Stamps right side (source) vector, harmonics domain
  void mnaCompApplyRightSideVectorStampHarm(Matrix &sourceVector) override;

  // #### MNA Parent Functions ####
  virtual void mnaParentInitialize(Real omega, Real timeStep,
                                   Attribute<Matrix>::Ptr leftVector){
      // By default, the parent has no custom initialization beyond what is done in CompositePowerComp::mnaCompInitialize
  };
  virtual void mnaParentApplySystemMatrixStamp(SparseMatrixRow &systemMatrix){
      // By default, the parent has no custom stamp on the system matrix, only the subcomponents are stamped
  };
  virtual void mnaParentApplyRightSideVectorStamp(Matrix &rightVector){
      // By default, the parent has no custom stamp on the right vector, only the subcomponents are stamped
  };
  virtual void mnaParentPreStep(Real time, Int timeStepCount){
      // By default, the parent has no custom pre-step, only the subcomponents' pre-steps are executed
  };
  virtual void mnaParentPostStep(Real time, Int timeStepCount,
                                 Attribute<Matrix>::Ptr &leftVector){
      // By default, the parent has no custom post-step, only the subcomponents' post-steps are executed
  };
  virtual void
  mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                  AttributeBase::List &attributeDependencies,
                                  AttributeBase::List &modifiedAttributes){
      // By default, the parent has no custom pre-step-dependencies, only the subcomponents' dependencies are added
  };
  virtual void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector){
      // By default, the parent has no custom post-step-dependencies, only the subcomponents' dependencies are added
  };

  // #### MNA Parent Functions, harmonics domain ####
  virtual void
  mnaParentInitializeHarm(Real omega, Real timeStep,
                          std::vector<Attribute<Matrix>::Ptr> leftVectors){};
  virtual void
  mnaParentApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                      Int freqIdx){};
  virtual void mnaParentApplyRightSideVectorStampHarm(Matrix &rightVector){};
  virtual void mnaParentPreStepHarm(Real time, Int timeStepCount){};
  virtual void
  mnaParentPostStepHarm(Real time, Int timeStepCount,
                        std::vector<Attribute<Matrix>::Ptr> &leftVectors){};

  class MnaPreStepHarm : public Task {
  public:
    explicit MnaPreStepHarm(CompositePowerComp<VarType> &comp)
        : Task(**comp.mName + ".MnaPreStepHarm"), mComp(comp) {
      comp.mnaAddPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) override {
      mComp.mnaCompPreStepHarm(time, timeStepCount);
    }

  private:
    CompositePowerComp<VarType> &mComp;
  };

  class MnaPostStepHarm : public Task {
  public:
    MnaPostStepHarm(CompositePowerComp<VarType> &comp,
                    const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
        : Task(**comp.mName + ".MnaPostStepHarm"), mComp(comp),
          mLeftVectors(leftVectors) {
      comp.mnaAddPostStepDependencies(mPrevStepDependencies,
                                      mAttributeDependencies,
                                      mModifiedAttributes, mLeftVectors[0]);
      for (UInt i = 1; i < mLeftVectors.size(); i++)
        mAttributeDependencies.push_back(mLeftVectors[i]);
    }
    void execute(Real time, Int timeStepCount) override {
      mComp.mnaCompPostStepHarm(time, timeStepCount, mLeftVectors);
    }

  private:
    CompositePowerComp<VarType> &mComp;
    std::vector<Attribute<Matrix>::Ptr> mLeftVectors;
  };

protected:
  /// Refreshes each subcomponent's own right-vector contribution, then calls
  /// mnaParentPreStepHarm (which typically sums them via
  /// mnaCompApplyRightSideVectorStampHarm, mirroring mnaParentPreStep).
  void mnaCompPreStepHarm(Real time, Int timeStepCount);
  /// Drives subcomponents' post-step voltage/current update (respecting
  /// before/after-parent order), then calls mnaParentPostStepHarm.
  void mnaCompPostStepHarm(Real time, Int timeStepCount,
                           std::vector<Attribute<Matrix>::Ptr> &leftVectors);
};
} // namespace CPS
