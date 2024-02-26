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

#include <cuda_runtime.h>
#include <cusolverDn.h>

#define CUDA_ERROR_HANDLER(func)                                               \
  {                                                                            \
    cudaError_t error;                                                         \
    if ((error = func) != cudaSuccess)                                         \
      std::cerr << cudaGetErrorString(error) << std::endl;                     \
  }

namespace DPsim {
class GpuDenseAdapter : public DirectLinearSolver {
protected:
  // #### Attributes required for GPU ####
  /// Solver-Handle
  cusolverDnHandle_t mCusolverHandle = nullptr;
  /// Stream
  cudaStream_t mStream = nullptr;

  /// Variables for solving one Equation-system (All pointer are device-pointer)
  struct GpuData {
    /// Device copy of System-Matrix
    double *matrix;
    /// Size of one dimension
    UInt size;
    /// Device copy of Vector
    double *vector;

    /// Device-Workspace for getrf
    double *workSpace;
    /// Pivoting-Sequence
    int *pivSeq;
    /// Errorinfo
    int *errInfo;
  } mDeviceCopy;

  void allocateDeviceMemory();

  void copySystemMatrixToDevice(Matrix systemMatrix);

  void LUfactorization();

public:
  /// Constructor with logging
  using DirectLinearSolver::DirectLinearSolver;

  /// Destructor
  virtual ~GpuDenseAdapter();

  /// Constructor
  GpuDenseAdapter();

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
