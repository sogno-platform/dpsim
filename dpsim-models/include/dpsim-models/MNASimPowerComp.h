// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/MNAStampUtils.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {

/// Base class for all MNA components that are transmitting power.
template <typename VarType>
class MNASimPowerComp : public SimPowerComp<VarType>, public MNAInterface {

private:
  Bool mHasPreStep;
  Bool mHasPostStep;

public:
  using Type = VarType;
  using Ptr = std::shared_ptr<MNASimPowerComp<VarType>>;
  using List = std::vector<Ptr>;

  /// This component's contribution ("stamp") to the right-side vector.
  /// TODO performance improvements from a sparse representation, at least during copying / summation?
  Attribute<Matrix>::Ptr mRightVector;

  /// List of tasks that relate to using MNA for this component (usually pre-step and/or post-step)
  Task::List mMnaTasks;

  /// Basic constructor that takes UID, name and log level
  MNASimPowerComp(String uid, String name, Bool hasPreStep, Bool hasPostStep,
                  Logger::Level logLevel)
      : SimPowerComp<VarType>(uid, name, logLevel), mHasPreStep(hasPreStep),
        mHasPostStep(hasPostStep),
        mRightVector(IdentifiedObject::mAttributes->createDynamic<Matrix>(
            "right_vector")){};

  /// Basic constructor that takes name and log level and sets the UID to name as well
  explicit MNASimPowerComp(String name, Bool hasPreStep = true,
                           Bool hasPostStep = true,
                           Logger::Level logLevel = Logger::Level::off)
      : MNASimPowerComp<VarType>(name, name, hasPreStep, hasPostStep,
                                 logLevel){};

  /// Destructor - does not do anything
  virtual ~MNASimPowerComp() = default;

  // Implementation of MNAInterface methods
  void mnaInitialize(Real omega, Real timeStep) final;
  void mnaInitialize(Real omega, Real timeStep,
                     Attribute<Matrix>::Ptr leftVector) final;
  void mnaApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) final;
  void mnaApplyRightSideVectorStamp(Matrix &rightVector) final;
  void mnaUpdateVoltage(const Matrix &leftVector) final;
  void mnaUpdateCurrent(const Matrix &leftVector) final;
  void mnaPreStep(Real time, Int timeStepCount) final;
  void mnaPostStep(Real time, Int timeStepCount,
                   Attribute<Matrix>::Ptr &leftVector) final;
  void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes) final;
  void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                  AttributeBase::List &attributeDependencies,
                                  AttributeBase::List &modifiedAttributes,
                                  Attribute<Matrix>::Ptr &leftVector) final;
  void mnaInitializeHarm(Real omega, Real timeStep,
                         std::vector<Attribute<Matrix>::Ptr> leftVector) final;
  void mnaApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                     Int freqIdx) final;
  void mnaApplyRightSideVectorStampHarm(Matrix &sourceVector) final;
  void mnaApplyRightSideVectorStampHarm(Matrix &sourceVector,
                                        Int freqIdx) final;

  // MNA Interface methods that can be overridden by components
  virtual void mnaCompInitialize(Real omega, Real timeStep,
                                 Attribute<Matrix>::Ptr leftVector);
  virtual void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  virtual void mnaCompApplyRightSideVectorStamp(Matrix &rightVector);
  virtual void mnaCompUpdateVoltage(const Matrix &leftVector);
  virtual void mnaCompUpdateCurrent(const Matrix &leftVector);
  virtual void mnaCompPreStep(Real time, Int timeStepCount);
  virtual void mnaCompPostStep(Real time, Int timeStepCount,
                               Attribute<Matrix>::Ptr &leftVector);
  virtual void
  mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                AttributeBase::List &attributeDependencies,
                                AttributeBase::List &modifiedAttributes);
  virtual void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector);
  virtual void
  mnaCompInitializeHarm(Real omega, Real timeStep,
                        std::vector<Attribute<Matrix>::Ptr> leftVector);
  virtual void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                                 Int freqIdx);
  virtual void mnaCompApplyRightSideVectorStampHarm(Matrix &sourceVector);
  virtual void mnaCompApplyRightSideVectorStampHarm(Matrix &sourceVector,
                                                    Int freqIdx);

  const Task::List &mnaTasks() const final;
  Attribute<Matrix>::Ptr getRightVector() const final;

  class MnaPreStep : public CPS::Task {
  public:
    explicit MnaPreStep(MNASimPowerComp<VarType> &comp)
        : Task(**comp.mName + ".MnaPreStep"), mComp(comp) {
      mComp.mnaAddPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) override {
      mComp.mnaPreStep(time, timeStepCount);
    };

  private:
    MNASimPowerComp<VarType> &mComp;
  };

  class MnaPostStep : public CPS::Task {
  public:
    MnaPostStep(MNASimPowerComp<VarType> &comp,
                Attribute<Matrix>::Ptr leftVector)
        : Task(**comp.mName + ".MnaPostStep"), mComp(comp),
          mLeftVector(leftVector) {
      mComp.mnaAddPostStepDependencies(mPrevStepDependencies,
                                       mAttributeDependencies,
                                       mModifiedAttributes, mLeftVector);
    }
    void execute(Real time, Int timeStepCount) override {
      mComp.mnaPostStep(time, timeStepCount, mLeftVector);
    };

  private:
    MNASimPowerComp<VarType> &mComp;
    Attribute<Matrix>::Ptr mLeftVector;
  };
};
} // namespace CPS
