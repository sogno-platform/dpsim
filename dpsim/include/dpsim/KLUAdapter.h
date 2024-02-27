/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

extern "C" {
#include <klu.h>
}

#include <bitset>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/DirectLinearSolver.h>

namespace DPsim {
class KLUAdapter : public DirectLinearSolver {
  /// Vector of variable entries in system matrix
  std::vector<std::pair<UInt, UInt>> mChangedEntries;

  /// Store variable columns in system matrix
  std::vector<Int> mVaryingColumns;
  /// Store variable rows in system matrix
  std::vector<Int> mVaryingRows;

  /// KLU-specific structs
  klu_common mCommon;
  klu_numeric *mNumeric = nullptr;
  klu_symbolic *mSymbolic = nullptr;

  /// Flags to indicate mode of operation
  /// Define which ordering to choose in preprocessing
  /// AMD_ORDERING is defined in SuiteSparse/AMD
  int mPreordering = AMD_ORDERING;

  /// Count Pivot faults
  int mPivotFaults = 0;

  PARTIAL_REFACTORIZATION_METHOD mPartialRefactorizationMethod =
      PARTIAL_REFACTORIZATION_METHOD::FACTORIZATION_PATH;

  /// Temporary value to store the number of nonzeros
  Int nnz;

public:
  /// Destructor
  ~KLUAdapter() override;

  /// Constructor
  KLUAdapter();

  /// Constructor with logging
  KLUAdapter(CPS::Logger::Log log);

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

protected:
  /// Function to print matrix in MatrixMarket's coo format
  void printMatrixMarket(SparseMatrix &systemMatrix, int counter) const;

  /// Apply configuration
  void applyConfiguration() override;
};
} // namespace DPsim
