/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/DenseLUAdapter.h>

#include <algorithm>

using namespace DPsim;

namespace DPsim {

DenseLUAdapter::~DenseLUAdapter() = default;

void DenseLUAdapter::applyConfiguration() {
  mUseStaticPreconditioner = mConfiguration.getStaticPreconditioner();
}

void DenseLUAdapter::preprocessing(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  (void)systemMatrix;
  mVariableEntries = listVariableSystemMatrixEntries;
  std::sort(mVariableEntries.begin(), mVariableEntries.end());
  mVariableEntries.erase(
      std::unique(mVariableEntries.begin(), mVariableEntries.end()),
      mVariableEntries.end());
}

void DenseLUAdapter::factorize(SparseMatrix &systemMatrix) {
  mReferenceMatrix = Matrix(systemMatrix);
  mCurrentMatrix = mReferenceMatrix;
  mResidual.resize(mReferenceMatrix.rows(), 1);
  mCorrection.resize(mReferenceMatrix.rows(), 1);
  LUFactorized.compute(mReferenceMatrix);
  mStaticPreconditionerActive =
      mUseStaticPreconditioner && !mVariableEntries.empty();
  // For small dense systems, applying the fixed inverse with a matrix-vector
  // product is cheaper than repeating two triangular substitutions for every
  // iterative correction. Keep the LU factors as the reference factorization
  // and exact fallback; only the correction application uses this cached map.
  constexpr Eigen::Index maxInverseCorrectionDimension = 256;
  mUseInverseForCorrections =
      mStaticPreconditionerActive &&
      mReferenceMatrix.rows() <= maxInverseCorrectionDimension;
  if (mUseInverseForCorrections) {
    const Matrix identity =
        Matrix::Identity(mReferenceMatrix.rows(), mReferenceMatrix.cols());
    mCorrectionInverse = LUFactorized.solve(identity);
  } else {
    mCorrectionInverse.resize(0, 0);
  }
}

void DenseLUAdapter::refactorize(SparseMatrix &systemMatrix) {
  if (!mStaticPreconditionerActive) {
    LUFactorized.compute(Matrix(systemMatrix));
    return;
  }
  // Only declared variable entries can differ from the reference matrix.
  // Keep all static dense coefficients in place instead of copying the whole
  // reduced matrix before updating the generator stamps.
  for (const auto &[row, column] : mVariableEntries)
    mCurrentMatrix(row, column) = systemMatrix.coeff(row, column);
}

void DenseLUAdapter::partialRefactorize(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  if (!mStaticPreconditionerActive) {
    LUFactorized.compute(Matrix(systemMatrix));
    return;
  }
  mVariableEntries = listVariableSystemMatrixEntries;
  std::sort(mVariableEntries.begin(), mVariableEntries.end());
  mVariableEntries.erase(
      std::unique(mVariableEntries.begin(), mVariableEntries.end()),
      mVariableEntries.end());
  for (const auto &[row, column] : mVariableEntries)
    mCurrentMatrix(row, column) = systemMatrix.coeff(row, column);
}

Matrix DenseLUAdapter::solve(Matrix &mRightHandSideVector) {
  Matrix solution;
  solveInto(mRightHandSideVector, solution);
  return solution;
}

void DenseLUAdapter::solveInto(Matrix &rightSideVector, Matrix &solution) {
  using Clock = std::chrono::steady_clock;
  ++mSolveCalls;
  if (!mStaticPreconditionerActive) {
    const auto start = mDetailedProfiling ? Clock::now() : Clock::time_point{};
    solution = LUFactorized.solve(rightSideVector);
    if (mDetailedProfiling) {
      mInitialSolveTime.total +=
          std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                               start);
      ++mInitialSolveTime.calls;
    }
    return;
  }

  auto start = mDetailedProfiling ? Clock::now() : Clock::time_point{};
  solution = LUFactorized.solve(rightSideVector);
  if (mDetailedProfiling) {
    mInitialSolveTime.total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start);
    ++mInitialSolveTime.calls;
  }

  start = mDetailedProfiling ? Clock::now() : Clock::time_point{};
  const Real rhsScale =
      std::max<Real>(1., rightSideVector.cwiseAbs().maxCoeff());
  const Real tolerance = 1e-12 * rhsScale;
  if (mUseInverseForCorrections) {
    // The cached inverse is a floating-point approximation to A0^-1. Form
    // the actual current-system residual so its solve error is not lost by
    // the simplified defect-correction recurrence used with triangular LU
    // solves.
    mResidual.noalias() = rightSideVector - mReferenceMatrix * solution;
    for (const auto &[row, column] : mVariableEntries) {
      const Real delta =
          mCurrentMatrix(row, column) - mReferenceMatrix(row, column);
      mResidual(row) -= delta * solution(column);
    }
  } else {
    mResidual.setZero();
    for (const auto &[row, column] : mVariableEntries) {
      const Real delta =
          mCurrentMatrix(row, column) - mReferenceMatrix(row, column);
      mResidual(row) -= delta * solution(column);
    }
  }
  if (mDetailedProfiling) {
    mResidualBuildTime.total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start);
    ++mResidualBuildTime.calls;
  }

  std::size_t solveIterations = 0;
  for (UInt iteration = 0; iteration < 12; ++iteration) {
    if (mResidual.cwiseAbs().maxCoeff() <= tolerance) {
      mMaximumRefinementIterations =
          std::max(mMaximumRefinementIterations, solveIterations);
      return;
    }
    ++solveIterations;
    ++mRefinementIterations;
    start = mDetailedProfiling ? Clock::now() : Clock::time_point{};
    if (mUseInverseForCorrections)
      mCorrection.noalias() = mCorrectionInverse * mResidual;
    else
      mCorrection = LUFactorized.solve(mResidual);
    if (mDetailedProfiling) {
      mCorrectionSolveTime.total +=
          std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                               start);
      ++mCorrectionSolveTime.calls;
    }
    start = mDetailedProfiling ? Clock::now() : Clock::time_point{};
    solution += mCorrection;
    if (mUseInverseForCorrections) {
      mResidual.noalias() -= mReferenceMatrix * mCorrection;
      for (const auto &[row, column] : mVariableEntries) {
        const Real delta =
            mCurrentMatrix(row, column) - mReferenceMatrix(row, column);
        mResidual(row) -= delta * mCorrection(column);
      }
    } else {
      mResidual.setZero();
      for (const auto &[row, column] : mVariableEntries) {
        const Real delta =
            mCurrentMatrix(row, column) - mReferenceMatrix(row, column);
        mResidual(row) -= delta * mCorrection(column);
      }
    }
    if (mDetailedProfiling) {
      mResidualUpdateTime.total +=
          std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                               start);
      ++mResidualUpdateTime.calls;
    }
  }

  // Refinement is an optimization only. Fall back to an exact solve if the
  // previous LU is not an adequate preconditioner for the new matrix.
  ++mFallbackCount;
  mMaximumRefinementIterations =
      std::max(mMaximumRefinementIterations, solveIterations);
  start = mDetailedProfiling ? Clock::now() : Clock::time_point{};
  LUFactorized.compute(mCurrentMatrix);
  if (mDetailedProfiling) {
    mFallbackFactorizationTime.total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start);
    ++mFallbackFactorizationTime.calls;
  }
  mReferenceMatrix = mCurrentMatrix;
  if (mUseInverseForCorrections) {
    const Matrix identity =
        Matrix::Identity(mReferenceMatrix.rows(), mReferenceMatrix.cols());
    mCorrectionInverse = LUFactorized.solve(identity);
  }
  start = mDetailedProfiling ? Clock::now() : Clock::time_point{};
  solution = LUFactorized.solve(rightSideVector);
  if (mDetailedProfiling) {
    mFallbackSolveTime.total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start);
    ++mFallbackSolveTime.calls;
  }
}

void DenseLUAdapter::writeDetailedProfile(std::ostream &profile) const {
  const auto write = [&profile](const char *name, const Timing &timing) {
    const double total = timing.total.count() * 1e-9;
    const double averageUs =
        timing.calls ? timing.total.count() * 1e-3 / timing.calls : 0.;
    profile << name << "," << total << "," << averageUs << ",0,0,"
            << timing.calls << "\n";
  };
  write("dense_lu_initial_solve", mInitialSolveTime);
  write("dense_lu_residual_build", mResidualBuildTime);
  write("dense_lu_correction_solves", mCorrectionSolveTime);
  write("dense_lu_residual_update", mResidualUpdateTime);
  write("dense_lu_fallback_factorization", mFallbackFactorizationTime);
  write("dense_lu_fallback_solve", mFallbackSolveTime);
}

void DenseLUAdapter::writeDetailedDiagnostics(std::ostream &diagnostics) const {
  diagnostics << "metric,value,unit\n"
              << "solve_calls," << mSolveCalls << ",calls\n"
              << "refinement_iterations_total," << mRefinementIterations
              << ",iterations\n"
              << "refinement_iterations_average_per_step,"
              << (mSolveCalls
                      ? static_cast<double>(mRefinementIterations) / mSolveCalls
                      : 0.)
              << ",iterations/solve\n"
              << "refinement_iterations_max_per_step,"
              << mMaximumRefinementIterations << ",iterations\n"
              << "inverse_correction_operator_selected,"
              << (mUseInverseForCorrections ? 1 : 0) << ",bool\n"
              << "fallback_refactorizations," << mFallbackCount << ",calls\n";
}
} // namespace DPsim
