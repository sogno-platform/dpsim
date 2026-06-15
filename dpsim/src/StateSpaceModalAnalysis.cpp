// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <Eigen/Eigenvalues>
#include <cmath>
#include <dpsim/StateSpaceModalAnalysis.h>
#include <limits>
#include <stdexcept>

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
    return;
  }

  Eigen::EigenSolver<Matrix> eigenSolver(Ad, false);

  if (eigenSolver.info() != Eigen::Success)
    throw std::runtime_error(
        "StateSpaceModalAnalysis: eigenvalue computation failed.");

  mDiscreteEigenvalues = eigenSolver.eigenvalues();

  mContinuousEigenvalues.resize(mDiscreteEigenvalues.rows());

  for (Eigen::Index idx = 0; idx < mDiscreteEigenvalues.rows(); ++idx)
    mContinuousEigenvalues(idx) =
        mapDiscreteToContinuous(mDiscreteEigenvalues(idx));
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

  for (const auto &abcBlock : mExtractor.getMetadata().abcStateIndexTriples) {
    for (UInt row = 0; row < 3; ++row) {
      for (UInt col = 0; col < 3; ++col) {
        transform(abcBlock[row], abcBlock[col]) = park(row, col);
      }
    }
  }

  return transform;
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
