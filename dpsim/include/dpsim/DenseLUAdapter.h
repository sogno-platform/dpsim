/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <bitset>
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
};
} // namespace DPsim
