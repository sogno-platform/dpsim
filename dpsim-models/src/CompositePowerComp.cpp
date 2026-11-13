// SPDX-License-Identifier: Apache-2.0

#include <dpsim-models/CompositePowerComp.h>

#include <algorithm>
#include <type_traits>

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
      this->mRightVectorStampComponents.push_back(subc);
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

  // RHS stamps are dense vectors for historical reasons, but a subcomponent
  // can only write at its connected terminal and virtual-node indices. Cache
  // those locations once so the composite can aggregate the stamps without
  // clearing and adding full network-sized vectors on every step.
  const UInt vectorSize = static_cast<UInt>(leftVector->get().rows());
  const UInt baseRowLimit = [&]() {
    if constexpr (std::is_same_v<VarType, Complex>)
      return vectorSize / (2 * this->mNumFreqs);
    else
      return vectorSize;
  }();
  auto appendNodeIndices = [baseRowLimit](SimPowerComp<VarType> &component,
                                          std::vector<UInt> &indices) {
    for (UInt terminal = 0; terminal < component.terminalNumberConnected();
         ++terminal) {
      if (component.node(terminal)->isGround())
        continue;
      for (UInt index : component.matrixNodeIndices(terminal)) {
        if (index < baseRowLimit)
          indices.push_back(index);
      }
    }
    for (UInt virtualNode = 0; virtualNode < component.virtualNodesNumber();
         ++virtualNode) {
      for (UInt index : component.virtualMatrixNodeIndices(virtualNode)) {
        if (index < baseRowLimit)
          indices.push_back(index);
      }
    }
    // A nested composite may contribute at its children's internal virtual
    // nodes as well as at its own terminals. Its MNA initialization has
    // already cached those rows before this parent is initialized.
    if (auto nested = dynamic_cast<CompositePowerComp<VarType> *>(&component))
      indices.insert(indices.end(), nested->mRightVectorAffectedIndices.begin(),
                     nested->mRightVectorAffectedIndices.end());
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  };

  mRightVectorStampIndices.clear();
  mRightVectorStampIndices.reserve(mRightVectorStampComponents.size());
  mRightVectorAffectedIndices.clear();
  appendNodeIndices(*this, mRightVectorAffectedIndices);
  for (const auto &component : mRightVectorStampComponents) {
    std::vector<UInt> indices;
    appendNodeIndices(*component, indices);
    mRightVectorAffectedIndices.insert(mRightVectorAffectedIndices.end(),
                                       indices.begin(), indices.end());
    mRightVectorStampIndices.push_back(std::move(indices));
  }
  std::sort(mRightVectorAffectedIndices.begin(),
            mRightVectorAffectedIndices.end());
  mRightVectorAffectedIndices.erase(
      std::unique(mRightVectorAffectedIndices.begin(),
                  mRightVectorAffectedIndices.end()),
      mRightVectorAffectedIndices.end());
  mRightVectorStampIndicesInitialized = true;

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
  if (!mRightVectorStampIndicesInitialized) {
    rightVector.setZero();
    for (const auto &stamp : mRightVectorStamps) {
      if ((**stamp).size() != 0)
        rightVector += **stamp;
    }
  } else {
    if constexpr (std::is_same_v<VarType, Complex>) {
      for (UInt freq = 0; freq < this->mNumFreqs; ++freq)
        for (UInt index : mRightVectorAffectedIndices)
          Math::setVectorElement(rightVector, index, Complex(0., 0.),
                                 this->mNumFreqs, freq);
    } else {
      for (UInt index : mRightVectorAffectedIndices)
        rightVector(index, 0) = 0.;
    }
    for (std::size_t stampIndex = 0; stampIndex < mRightVectorStamps.size();
         ++stampIndex) {
      const auto &stamp = mRightVectorStamps[stampIndex];
      if ((**stamp).size() == 0)
        continue;
      if constexpr (std::is_same_v<VarType, Complex>) {
        for (UInt freq = 0; freq < this->mNumFreqs; ++freq)
          for (UInt index : mRightVectorStampIndices[stampIndex])
            Math::addToVectorElement(rightVector, index,
                                     Math::complexFromVectorElement(
                                         **stamp, index, this->mNumFreqs, freq),
                                     this->mNumFreqs, freq);
      } else {
        for (UInt index : mRightVectorStampIndices[stampIndex])
          rightVector(index, 0) += (**stamp)(index, 0);
      }
    }
  }
  mnaParentApplyRightSideVectorStamp(rightVector);
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (!mRightVectorStampIndicesInitialized) {
    rightVector += **this->mRightVector;
    return;
  }

  const Matrix &stamp = **this->mRightVector;
  if constexpr (std::is_same_v<VarType, Complex>) {
    for (UInt freq = 0; freq < this->mNumFreqs; ++freq)
      for (UInt index : mRightVectorAffectedIndices)
        Math::addToVectorElement(
            rightVector, index,
            Math::complexFromVectorElement(stamp, index, this->mNumFreqs, freq),
            this->mNumFreqs, freq);
  } else {
    for (UInt index : mRightVectorAffectedIndices)
      rightVector(index, 0) += stamp(index, 0);
  }
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompPreStep(Real time, Int timeStepCount) {
  for (const auto &subComp : mSubcomponentsPreStepBeforeParent) {
    subComp->mnaPreStep(time, timeStepCount);
  }
  mnaParentPreStep(time, timeStepCount);
  for (const auto &subComp : mSubcomponentsPreStepAfterParent) {
    subComp->mnaPreStep(time, timeStepCount);
  }
}

template <typename VarType>
void CompositePowerComp<VarType>::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  for (const auto &subComp : mSubcomponentsPostStepBeforeParent) {
    subComp->mnaPostStep(time, timeStepCount, leftVector);
  }
  mnaParentPostStep(time, timeStepCount, leftVector);
  for (const auto &subComp : mSubcomponentsPostStepAfterParent) {
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

// Declare specializations to move definitions to .cpp
template class CPS::CompositePowerComp<Real>;
template class CPS::CompositePowerComp<Complex>;
