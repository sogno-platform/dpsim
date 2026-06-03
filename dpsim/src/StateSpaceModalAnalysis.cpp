// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <Eigen/Eigenvalues>
#include <dpsim/StateSpaceModalAnalysis.h>
#include <stdexcept>

namespace DPsim {

StateSpaceModalAnalysis::StateSpaceModalAnalysis(
    const MNAStateSpaceExtractor &extractor)
    : mExtractor(extractor) {}

void StateSpaceModalAnalysis::update() {
  if (!mExtractor.isInitialized())
    throw std::logic_error("StateSpaceModalAnalysis requires an initialized "
                           "MNAStateSpaceExtractor.");

  const Matrix &Ad = mExtractor.getDiscreteStateMatrix();

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

CPS::Complex
StateSpaceModalAnalysis::mapDiscreteToContinuous(const CPS::Complex &z) const {
  const CPS::Complex one(1.0, 0.0);
  const CPS::Complex denominator = z + one;

  if (std::abs(denominator) <= DOUBLE_EPSILON)
    return CPS::Complex(std::numeric_limits<Real>::infinity(), 0.0);

  return (2.0 / mExtractor.getTimeStep()) * (z - one) / denominator;
}

} // namespace DPsim
