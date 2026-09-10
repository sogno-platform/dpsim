// SPDX-License-Identifier: Apache-2.0

#include <dpsim-models/CompositePowerComp.h>

using namespace CPS;

template <typename VarType>
void CompositePowerComp<VarType>::initializeFromNodesAndTerminals(
    Real frequency) {
  this->createSubComponents();

  initializeParentFromNodesAndTerminals(frequency);

  for (auto subComp : this->mSubComponents) {
    subComp->initialize(this->mFrequencies);
    subComp->initializeFromNodesAndTerminals(frequency);
  }
}

template <typename VarType>
void CompositePowerComp<VarType>::addMNASubComponent(
    typename SimPowerComp<VarType>::Ptr subc,
    MNA_SUBCOMP_TASK_ORDER preStepOrder, MNA_SUBCOMP_TASK_ORDER postStepOrder,
    Bool contributeToRightVector) {
  this->mSubComponents.push_back(subc);
  if (auto mnasubcomp =
          std::dynamic_pointer_cast<MNASimPowerComp<VarType>>(subc)) {
    this->mSubcomponentsMNA.push_back(mnasubcomp);

    if (contributeToRightVector) {
      this->mRightVectorStamps.push_back(mnasubcomp->mRightVector);
    }

    switch (preStepOrder) {
    case MNA_SUBCOMP_TASK_ORDER::NO_TASK:
      break;
    case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
      this->mSubcomponentsPreStepBeforeParent.push_back(mnasubcomp);
      break;
    }
    case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
      this->mSubcomponentsPreStepAfterParent.push_back(mnasubcomp);
      break;
    }
    }
    switch (postStepOrder) {
    case MNA_SUBCOMP_TASK_ORDER::NO_TASK:
      break;
    case MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT: {
      this->mSubcomponentsPostStepBeforeParent.push_back(mnasubcomp);
      break;
    }
    case MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT: {
      this->mSubcomponentsPostStepAfterParent.push_back(mnasubcomp);
      break;
    }
    }
  }
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  SimPowerComp<VarType>::updateMatrixNodeIndices();

  for (auto subComp : mSubcomponentsMNA) {
    subComp->mnaInitialize(omega, timeStep, leftVector);
  }

  **this->mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

  mnaParentInitialize(omega, timeStep, leftVector);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (auto subComp : mSubcomponentsMNA) {
    subComp->mnaApplySystemMatrixStamp(systemMatrix);
  }
  mnaParentApplySystemMatrixStamp(systemMatrix);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  rightVector.setZero();
  for (auto stamp : mRightVectorStamps) {
    if ((**stamp).size() != 0) {
      rightVector += **stamp;
    }
  }
  mnaParentApplyRightSideVectorStamp(rightVector);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompPreStep(Real time, Int timeStepCount) {
  for (auto subComp : mSubcomponentsPreStepBeforeParent) {
    subComp->mnaPreStep(time, timeStepCount);
  }
  mnaParentPreStep(time, timeStepCount);
  for (auto subComp : mSubcomponentsPreStepAfterParent) {
    subComp->mnaPreStep(time, timeStepCount);
  }
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  for (auto subComp : mSubcomponentsPostStepBeforeParent) {
    subComp->mnaPostStep(time, timeStepCount, leftVector);
  }
  mnaParentPostStep(time, timeStepCount, leftVector);
  for (auto subComp : mSubcomponentsPostStepAfterParent) {
    subComp->mnaPostStep(time, timeStepCount, leftVector);
  }
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  for (auto subComp : mSubcomponentsMNA) {
    subComp->mnaAddPreStepDependencies(
        prevStepDependencies, attributeDependencies, modifiedAttributes);
  }
  mnaParentAddPreStepDependencies(prevStepDependencies, attributeDependencies,
                                  modifiedAttributes);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  for (auto subComp : mSubcomponentsMNA) {
    subComp->mnaAddPostStepDependencies(prevStepDependencies,
                                        attributeDependencies,
                                        modifiedAttributes, leftVector);
  }
  mnaParentAddPostStepDependencies(prevStepDependencies, attributeDependencies,
                                   modifiedAttributes, leftVector);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompInitializeHarm(
    Real omega, Real timeStep,
    std::vector<Attribute<Matrix>::Ptr> leftVectors) {
  SimPowerComp<VarType>::updateMatrixNodeIndices();

  for (auto subComp : mSubcomponentsMNA) {
    subComp->mnaInitializeHarm(omega, timeStep, leftVectors);
  }

  **this->mRightVector =
      Matrix::Zero(leftVectors[0]->get().rows(), this->mNumFreqs);

  this->mMnaTasks.push_back(
      std::make_shared<typename CompositePowerComp<VarType>::MnaPreStepHarm>(
          *this));
  this->mMnaTasks.push_back(
      std::make_shared<typename CompositePowerComp<VarType>::MnaPostStepHarm>(
          *this, leftVectors));

  mnaParentInitializeHarm(omega, timeStep, leftVectors);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompApplySystemMatrixStampHarm(
    SparseMatrixRow &systemMatrix, Int freqIdx) {
  for (auto subComp : mSubcomponentsMNA) {
    subComp->mnaApplySystemMatrixStampHarm(systemMatrix, freqIdx);
  }
  mnaParentApplySystemMatrixStampHarm(systemMatrix, freqIdx);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompApplyRightSideVectorStampHarm(
    Matrix &sourceVector) {
  sourceVector.setZero();
  for (auto stamp : mRightVectorStamps) {
    if ((**stamp).size() != 0) {
      sourceVector += **stamp;
    }
  }
  mnaParentApplyRightSideVectorStampHarm(sourceVector);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompPreStepHarm(Real time,
                                                     Int timeStepCount) {
  for (auto subComp : mSubcomponentsMNA) {
    Matrix &rightVec = **subComp->getRightVector();
    if (rightVec.size() != 0)
      subComp->mnaApplyRightSideVectorStampHarm(rightVec);
  }
  mnaParentPreStepHarm(time, timeStepCount);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompPostStepHarm(
    Real time, Int timeStepCount,
    std::vector<Attribute<Matrix>::Ptr> &leftVectors) {
  for (auto subComp : mSubcomponentsPostStepBeforeParent) {
    for (UInt freq = 0; freq < this->mNumFreqs; freq++)
      subComp->mnaUpdateVoltageHarm(**leftVectors[freq], freq);
    for (UInt freq = 0; freq < this->mNumFreqs; freq++)
      subComp->mnaUpdateCurrentHarm(freq);
  }
  mnaParentPostStepHarm(time, timeStepCount, leftVectors);
  for (auto subComp : mSubcomponentsPostStepAfterParent) {
    for (UInt freq = 0; freq < this->mNumFreqs; freq++)
      subComp->mnaUpdateVoltageHarm(**leftVectors[freq], freq);
    for (UInt freq = 0; freq < this->mNumFreqs; freq++)
      subComp->mnaUpdateCurrentHarm(freq);
  }
}

// Declare specializations to move definitions to .cpp
template class CPS::CompositePowerComp<Real>;
template class CPS::CompositePowerComp<Complex>;
