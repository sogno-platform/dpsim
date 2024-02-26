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
  MNAInterface::List mSubcomponentsBeforePreStep;
  MNAInterface::List mSubcomponentsAfterPreStep;
  MNAInterface::List mSubcomponentsBeforePostStep;
  MNAInterface::List mSubcomponentsAfterPostStep;

  std::vector<CPS::Attribute<Matrix>::Ptr> mRightVectorStamps;

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
};
} // namespace CPS
