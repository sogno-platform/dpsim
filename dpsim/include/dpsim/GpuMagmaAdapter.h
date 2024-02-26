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

#include <cusolverSp.h>
#include <magma_v2.h>
#include <magmasparse.h>

namespace DPsim {
class GpuMagmaAdapter : public DirectLinearSolver {
protected:
  std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic>> mTransp;
  // #### Attributes required for GPU ####
  /// Solver-Handle
  magma_dopts mMagmaOpts;
  magma_queue_t mMagmaQueue;

  /// Systemmatrix
  magma_d_matrix mHostSysMat;
  magma_d_matrix mDevSysMat;

  /// RHS-Vector
  magma_d_matrix mHostRhsVec;
  magma_d_matrix mDevRhsVec;
  /// LHS-Vector
  magma_d_matrix mHostLhsVec;
  magma_d_matrix mDevLhsVec;

  // TODO: fix mSLog for solvers (all solvers)
  // using Solver::mSLog;

  void iluPreconditioner();

  void performFactorization(SparseMatrix &systemMatrix);

public:
  /// Constructor with logging
  using DirectLinearSolver::DirectLinearSolver;

  /// Destructor
  virtual ~GpuMagmaAdapter();

  /// Constructor
  GpuMagmaAdapter();

  /// preprocessing function pre-ordering and scaling the matrix
  virtual void preprocessing(SparseMatrix &systemMatrix,
                             std::vector<std::pair<UInt, UInt>>
                                 &listVariableSystemMatrixEntries) override;

  /// factorization function with partial pivoting
  virtual void factorize(SparseMatrix &systemMatrix) override;

  /// refactorization without partial pivoting
  virtual void refactorize(SparseMatrix &systemMatrix) override;

  /// partial refactorization withouth partial pivoting
  virtual void partialRefactorize(SparseMatrix &systemMatrix,
                                  std::vector<std::pair<UInt, UInt>> &
                                      listVariableSystemMatrixEntries) override;

  /// solution function for a right hand side
  virtual Matrix solve(Matrix &rightSideVector) override;
};
} // namespace DPsim
