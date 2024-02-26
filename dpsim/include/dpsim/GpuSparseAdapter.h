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
#include <dpsim/Cuda/CudaUtility.h>

namespace DPsim {
class GpuSparseAdapter : public DirectLinearSolver {
protected:
  // #### Attributes required for GPU ####
  /// Solver-Handle
  cusparseHandle_t mCusparsehandle = nullptr;
  cusolverSpHandle_t mCusolverhandle = nullptr;

  /// Systemmatrix on Device
  std::unique_ptr<cuda::CudaMatrix<double, int>> mSysMat = nullptr;
  std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic>> mTransp = nullptr;

  /// RHS-Vector
  cuda::Vector<double> mGpuRhsVec = 0;
  /// LHS-Vector
  cuda::Vector<double> mGpuLhsVec = 0;
  /// Intermediate Vector
  cuda::Vector<double> mGpuIntermediateVec = 0;

  void iluPreconditioner();

  void performFactorization(SparseMatrix &systemMatrix);

private:
  ///Required shared Variables
  cuda::Vector<char> pBuffer = 0;
  cusparseMatDescr_t descr_L = nullptr;
  cusparseMatDescr_t descr_U = nullptr;
  csrsv2Info_t info_L = nullptr;
  csrsv2Info_t info_U = nullptr;

  void checkCusparseStatus(cusparseStatus_t status,
                           std::string additionalInfo = "cuSparse Error:");

public:
  /// Constructor with logging
  using DirectLinearSolver::DirectLinearSolver;

  /// Constructor
  GpuSparseAdapter();

  /// Destructor
  virtual ~GpuSparseAdapter();

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
