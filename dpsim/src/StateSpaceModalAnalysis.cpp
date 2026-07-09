// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include <dpsim-models/MathUtils.h>
#include <dpsim/StateSpaceModalAnalysis.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace DPsim {

namespace {

Matrix parkTransformDQ0(Real theta) {
  Matrix transform(3, 3);

  const Real k = std::sqrt(2.0 / 3.0);
  const Real k0 = 1.0 / std::sqrt(3.0);

  transform.row(0) << k * std::cos(theta), k * std::cos(theta - 2.0 * PI / 3.0),
      k * std::cos(theta + 2.0 * PI / 3.0);

  transform.row(1) << -k * std::sin(theta),
      -k * std::sin(theta - 2.0 * PI / 3.0),
      -k * std::sin(theta + 2.0 * PI / 3.0);

  transform.row(2) << k0, k0, k0;

  return transform;
}

String fallbackStateName(UInt index) { return "x" + std::to_string(index); }

} // namespace

StateSpaceModalAnalysis::StateSpaceModalAnalysis(
    const MNAStateSpaceExtractor &extractor)
    : mExtractor(extractor) {}

void StateSpaceModalAnalysis::update() {
  if (!mExtractor.isInitialized())
    throw std::logic_error("StateSpaceModalAnalysis requires an initialized "
                           "MNAStateSpaceExtractor.");

  const Matrix Ad = buildDiscreteStateMatrixInAnalysisFrame();

  if (Ad.rows() == 0) {
    mDiscreteEigenvalues.resize(0);
    mContinuousEigenvalues.resize(0);

    mRightEigenvectors.resize(0, 0);
    mLeftEigenvectors.resize(0, 0);
    mParticipationFactors.resize(0, 0);

    mStateNames.clear();

    return;
  }

  if (Ad.rows() != Ad.cols())
    throw std::logic_error(
        "StateSpaceModalAnalysis requires a square state matrix.");

  mStateNames = buildStateNamesInAnalysisFrame();

  Eigen::EigenSolver<Matrix> eigenSolver(Ad, true);

  if (eigenSolver.info() != Eigen::Success)
    throw std::runtime_error(
        "StateSpaceModalAnalysis: eigenvalue computation failed.");

  mDiscreteEigenvalues = eigenSolver.eigenvalues();

  mContinuousEigenvalues.resize(mDiscreteEigenvalues.rows());

  for (Eigen::Index idx = 0; idx < mDiscreteEigenvalues.rows(); ++idx)
    mContinuousEigenvalues(idx) =
        mapDiscreteToContinuous(mDiscreteEigenvalues(idx));

  mRightEigenvectors = eigenSolver.eigenvectors();

  Eigen::FullPivLU<CPS::MatrixComp> eigenvectorLu(mRightEigenvectors);

  if (!eigenvectorLu.isInvertible())
    throw std::runtime_error(
        "StateSpaceModalAnalysis: cannot compute participation factors because "
        "the eigenvector matrix is singular.");

  mLeftEigenvectors = eigenvectorLu.inverse();

  mParticipationFactors = CPS::Math::elementwiseProduct(
      mRightEigenvectors, mLeftEigenvectors.transpose());
}

Matrix
StateSpaceModalAnalysis::buildDiscreteStateMatrixInAnalysisFrame() const {
  const Matrix &nativeAd = mExtractor.getDiscreteStateMatrix();

  if (mAnalysisFrame == StateSpaceAnalysisFrame::Native)
    return nativeAd;

  if (mAnalysisFrame == StateSpaceAnalysisFrame::GlobalDQ0) {
    if (!mExtractor.hasExtractionTime()) {
      throw std::logic_error(
          "GlobalDQ0 modal analysis requires a valid extraction timestamp.");
    }

    if (mGlobalOmega <= 0.0) {
      throw std::logic_error(
          "GlobalDQ0 modal analysis requires a positive frame angular speed.");
    }

    const Real time = mExtractor.getLastExtractionTime();
    const Real timeStep = mExtractor.getTimeStep();

    const Real thetaNow = mGlobalTheta0 + mGlobalOmega * time;
    const Real thetaNext = thetaNow + mGlobalOmega * timeStep;

    const Matrix transformNow = buildGlobalDq0Transformation(thetaNow);
    const Matrix transformNext = buildGlobalDq0Transformation(thetaNext);

    // For a time-dependent discrete coordinate transformation
    // xGlobalDq0[k] = T[k] xNative[k], the transformed transition matrix is
    // AdGlobalDq0[k] = T[k+1] AdNative[k] T[k]^{-1}.
    //
    // The Park transform is power-invariant, so T^{-1} = T^T.
    return transformNext * nativeAd * transformNow.transpose();
  }

  throw std::logic_error("Unsupported state-space analysis frame.");
}

Matrix StateSpaceModalAnalysis::buildGlobalDq0Transformation(Real theta) const {
  const UInt stateCount = mExtractor.getStateCount();

  Matrix transform = Matrix::Identity(stateCount, stateCount);

  const Matrix park = parkTransformDQ0(theta);

  for (const auto &abcBlock : mExtractor.getMetadata().abcStateBlocks) {
    for (UInt row = 0; row < 3; ++row) {
      for (UInt col = 0; col < 3; ++col) {
        transform(abcBlock.indices[row], abcBlock.indices[col]) =
            park(row, col);
      }
    }
  }

  return transform;
}

std::vector<String>
StateSpaceModalAnalysis::buildStateNamesInAnalysisFrame() const {
  const UInt stateCount = mExtractor.getStateCount();
  const auto &metadata = mExtractor.getMetadata();

  std::vector<String> stateNames(stateCount);

  for (UInt idx = 0; idx < stateCount; ++idx) {
    if (idx < metadata.stateNames.size() && !metadata.stateNames[idx].empty())
      stateNames[idx] = metadata.stateNames[idx];
    else
      stateNames[idx] = fallbackStateName(idx);
  }

  if (mAnalysisFrame == StateSpaceAnalysisFrame::Native)
    return stateNames;

  if (mAnalysisFrame == StateSpaceAnalysisFrame::GlobalDQ0) {
    for (const auto &abcBlock : metadata.abcStateBlocks) {
      if (abcBlock.name.empty())
        throw std::logic_error(
            "GlobalDQ0 modal analysis requires named abc state blocks.");

      stateNames[abcBlock.indices[0]] = abcBlock.name + "_d";
      stateNames[abcBlock.indices[1]] = abcBlock.name + "_q";
      stateNames[abcBlock.indices[2]] = abcBlock.name + "_0";
    }

    return stateNames;
  }

  throw std::logic_error("Unsupported state-space analysis frame.");
}

CPS::Complex
StateSpaceModalAnalysis::mapDiscreteToContinuous(const CPS::Complex &z) const {
  const CPS::Complex one(1.0, 0.0);
  const CPS::Complex denominator = z + one;

  if (std::abs(denominator) <= DOUBLE_EPSILON)
    return CPS::Complex(std::numeric_limits<Real>::infinity(), 0.0);

  return (2.0 / mExtractor.getTimeStep()) * (z - one) / denominator;
}

} // namespace DPsim
