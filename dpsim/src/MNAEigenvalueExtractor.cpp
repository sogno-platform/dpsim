#include <dpsim/MNAEigenvalueExtractor.h>

namespace DPsim {
template <typename VarType>
MNAEigenvalueExtractor<VarType>::MNAEigenvalueExtractor(
    CPS::Logger::Level logLevel)
    : mLogger("MNAEigenvalueExtractor", logLevel) {
  mEigenvalues = CPS::AttributeStatic<MatrixComp>::make();
  mDiscreteEigenvalues = CPS::AttributeStatic<MatrixComp>::make();
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::initialize(
    const CPS::SystemTopology &topology, UInt numMatrixNodeIndices,
    Real timeStep) {
  setParameters(topology, timeStep);
  identifyEigenvalueComponents(topology.mComponents);
  createEmptyEigenvalueMatrices(numMatrixNodeIndices);
  stampEigenvalueMatrices();
  mLogger.setLogAttributes(mEigenvalues, mDiscreteEigenvalues);
  mLogger.logInitialization(mSignMatrix, mDiscretizationMatrix,
                            mBranchNodeIncidenceMatrix,
                            mNodeBranchIncidenceMatrix);
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::setParameters(
    const CPS::SystemTopology &topology, Real timeStep) {
  mTimeStep = timeStep;
  // Relevant only for DP
  mSystemOmega = topology.mSystemOmega;

  Real denominator = mTimeStep * mTimeStep * mSystemOmega * mSystemOmega + 4.0;
  Real realPart =
      (4.0 - mTimeStep * mTimeStep * mSystemOmega * mSystemOmega) / denominator;
  Real imagPart = (-4.0 * mTimeStep * mSystemOmega) / denominator;
  mCoeffDP = Complex(realPart, imagPart);
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::identifyEigenvalueComponents(
    const CPS::IdentifiedObject::List &components) {
  // TODO: throw exception if topology contains components that do not implement EigenvalueCompInterface
  UInt branchIdx = 0;
  for (auto component : components) {
    auto eigenvalueComponent =
        std::dynamic_pointer_cast<CPS::EigenvalueCompInterface>(component);
    if (eigenvalueComponent) {
      mEigenvalueComponentToBranchIdx[eigenvalueComponent] = branchIdx;
      auto eigenvalueDynamicComponent = std::dynamic_pointer_cast<
          CPS::EigenvalueDynamicCompInterface<VarType>>(component);
      if (eigenvalueDynamicComponent) {
        mEigenvalueDynamicComponents.push_back(eigenvalueDynamicComponent);
      }
      branchIdx++;
    }
  }
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::createEmptyEigenvalueMatrices(
    UInt numMatrixNodeIndices) {
  int nBranches = mEigenvalueComponentToBranchIdx.size();
  mSignMatrix = MatrixVar<VarType>::Zero(nBranches, nBranches);
  mDiscretizationMatrix = MatrixVar<VarType>::Zero(nBranches, nBranches);
  mBranchNodeIncidenceMatrix = Matrix::Zero(nBranches, numMatrixNodeIndices);
  **mEigenvalues = MatrixComp::Zero(mEigenvalueDynamicComponents.size(), 1);
  **mDiscreteEigenvalues =
      MatrixComp::Zero(mEigenvalueDynamicComponents.size(), 1);
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::stampEigenvalueMatrices() {
  for (const auto &compToBranchIdx : mEigenvalueComponentToBranchIdx) {
    auto comp = compToBranchIdx.first;
    UInt branchIdx = compToBranchIdx.second;
    comp->stampBranchNodeIncidenceMatrix(branchIdx, mBranchNodeIncidenceMatrix);
  }
  mNodeBranchIncidenceMatrix = mBranchNodeIncidenceMatrix.transpose();
  for (const auto &dynamicComp : mEigenvalueDynamicComponents) {
    UInt branchIdx = mEigenvalueComponentToBranchIdx[dynamicComp];
    dynamicComp->stampSignMatrix(branchIdx, mSignMatrix, mCoeffDP);
    dynamicComp->stampDiscretizationMatrix(branchIdx, mDiscretizationMatrix,
                                           mCoeffDP);
  }
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::extractEigenvalues(
    const Matrix &powerSystemMatrix, Real time, Int timeStepCount) {
  calculateStateMatrix(powerSystemMatrix);
  computeDiscreteEigenvalues();
  recoverEigenvalues();
  mLogger.logExtraction(time, timeStepCount, mStateMatrix);
}

template <>
void MNAEigenvalueExtractor<Real>::calculateStateMatrix(
    const Matrix &powerSystemMatrix) {
  // TODO: use right hand side solving of factorized power system matrix instead of inversion (performance).
  Matrix intermediateResult =
      powerSystemMatrix.inverse() * mNodeBranchIncidenceMatrix;
  mStateMatrix = mSignMatrix - mDiscretizationMatrix *
                                   mBranchNodeIncidenceMatrix *
                                   intermediateResult;
}

template <>
void MNAEigenvalueExtractor<Complex>::calculateStateMatrix(
    const Matrix &powerSystemMatrix) {
  // TODO: use right hand side solving of factorized power system matrix instead of inversion (performance).
  MatrixComp compPowerSystemMatrix =
      CPS::Math::convertRealEquivalentToComplexMatrix(powerSystemMatrix);
  MatrixComp intermediateResult =
      compPowerSystemMatrix.inverse() * mNodeBranchIncidenceMatrix;
  mStateMatrix = mSignMatrix - mDiscretizationMatrix *
                                   mBranchNodeIncidenceMatrix *
                                   intermediateResult;
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::computeDiscreteEigenvalues() {
  auto discreteEigenvaluesIncludingZeros = mStateMatrix.eigenvalues();
  **mDiscreteEigenvalues =
      CPS::Math::returnNonZeroElements(discreteEigenvaluesIncludingZeros);
  // TODO: filter out eigenvalues = -1 + 0i to avoid division by zero in recoverEigenvalues().
}

template <> void MNAEigenvalueExtractor<Real>::recoverEigenvalues() {
  **mEigenvalues = 2.0 / mTimeStep * ((**mDiscreteEigenvalues).array() - 1.0) /
                   ((**mDiscreteEigenvalues).array() + 1.0);
}

template <> void MNAEigenvalueExtractor<Complex>::recoverEigenvalues() {
  **mEigenvalues = 2.0 / mTimeStep * ((**mDiscreteEigenvalues).array() - 1.0) /
                       ((**mDiscreteEigenvalues).array() + 1.0) +
                   Complex(0.0, 1.0) * mSystemOmega;
}

template <typename VarType>
void MNAEigenvalueExtractor<VarType>::closeLogger() {
  mLogger.close();
}

template class MNAEigenvalueExtractor<Real>;
template class MNAEigenvalueExtractor<Complex>;
} // namespace DPsim
