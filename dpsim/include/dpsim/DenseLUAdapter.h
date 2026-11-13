/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <bitset>
#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/DirectLinearSolver.h>

namespace DPsim {
class DenseLUAdapter : public DirectLinearSolver {
  Eigen::PartialPivLU<Matrix> LUFactorized;
  Matrix mReferenceMatrix;
  Matrix mCorrectionInverse;
  Matrix mCurrentMatrix;
  Matrix mResidual;
  Matrix mCorrection;
  std::vector<std::pair<UInt, UInt>> mVariableEntries;
  Bool mUseStaticPreconditioner = false;
  Bool mStaticPreconditionerActive = false;
  Bool mUseInverseForCorrections = false;
  Bool mDetailedProfiling = false;
  struct Timing {
    std::chrono::nanoseconds total{};
    std::size_t calls = 0;
  };
  Timing mInitialSolveTime, mResidualBuildTime, mCorrectionSolveTime,
      mResidualUpdateTime, mFallbackFactorizationTime, mFallbackSolveTime;
  std::size_t mSolveCalls = 0;
  std::size_t mRefinementIterations = 0;
  std::size_t mMaximumRefinementIterations = 0;
  std::size_t mFallbackCount = 0;

  void applyConfiguration() override;

public:
  /// Constructor with logging
  using DirectLinearSolver::DirectLinearSolver;

  /// Destructor
  ~DenseLUAdapter() override;

  /// preprocessing function pre-ordering and scaling the matrix
  void preprocessing(SparseMatrix &systemMatrix,
                     std::vector<std::pair<UInt, UInt>>
                         &listVariableSystemMatrixEntries) override;

  /// factorization function with partial pivoting
  void factorize(SparseMatrix &systemMatrix) override;

  /// refactorization without partial pivoting
  void refactorize(SparseMatrix &systemMatrix) override;

  /// partial refactorization withouth partial pivoting
  void partialRefactorize(SparseMatrix &systemMatrix,
                          std::vector<std::pair<UInt, UInt>>
                              &listVariableSystemMatrixEntries) override;

  /// solution function for a right hand side
  Matrix solve(Matrix &rightSideVector) override;
  void solveInto(Matrix &rightSideVector, Matrix &solution) override;
  void setDetailedProfiling(Bool enabled) { mDetailedProfiling = enabled; }
  void writeDetailedProfile(std::ostream &profile) const;
  void writeDetailedDiagnostics(std::ostream &diagnostics) const;
};
} // namespace DPsim
