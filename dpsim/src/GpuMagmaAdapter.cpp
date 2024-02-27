/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "magma_types.h"
#include <cusolverSp.h>
#include <dpsim/GpuMagmaAdapter.h>

using namespace DPsim;

namespace DPsim {
GpuMagmaAdapter::~GpuMagmaAdapter() {
  magma_dmfree(&mDevSysMat, mMagmaQueue);
  magma_dmfree(&mDevRhsVec, mMagmaQueue);
  magma_dmfree(&mDevLhsVec, mMagmaQueue);

  magma_queue_destroy(mMagmaQueue);
  magma_finalize();
}

void GpuMagmaAdapter::performFactorization(SparseMatrix &systemMatrix) {
  int size = systemMatrix.rows();
  int p_nnz = 0;
  int p[size];
  auto hMat = systemMatrix;
  size_t nnz = hMat.nonZeros();
  cusparseMatDescr_t descr_M = 0;

  cusolverSpHandle_t mCusolverhandle;
  if (cusolverSpCreate(&mCusolverhandle) != CUSOLVER_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "cusolverSpCreate returend an error");
    return;
  }
  if (cusparseCreateMatDescr(&descr_M) != CUSPARSE_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "cusolver returend an error");
    return;
  }

  if (cusparseSetMatIndexBase(descr_M, CUSPARSE_INDEX_BASE_ZERO) !=
      CUSPARSE_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "cusolver returend an error");
    return;
  }
  if (cusparseSetMatType(descr_M, CUSPARSE_MATRIX_TYPE_GENERAL) !=
      CUSPARSE_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "cusolver returend an error");
    return;
  }

  if (cusolverSpDcsrzfdHost(mCusolverhandle, size, nnz, descr_M,
                            hMat.valuePtr(), hMat.outerIndexPtr(),
                            hMat.innerIndexPtr(), p,
                            &p_nnz) != CUSOLVER_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "cusolverSpDcsrzfdHost returend an error");
    return;
  }

  // create Eigen::PermutationMatrix from the p
  mTransp = std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic>>(
      new Eigen::PermutationMatrix<Eigen::Dynamic>(
          Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, 1>>(p, size, 1)));

  // apply permutation
  //std::cout << "Before System Matrix:" << std::endl << hMat[0] << std::endl;
  hMat = *mTransp * hMat;
  //std::cout << "permutation:" << std::endl << mTransp->toDenseMatrix() << std::endl;
  //std::cout << "inverse permutation:" << std::endl << mTransp->inverse().toDenseMatrix() << std::endl;
  //std::cout << "System Matrix:" << std::endl << hMat[0] << std::endl;
  magma_dcsrset(size, size, hMat.outerIndexPtr(), hMat.innerIndexPtr(),
                hMat.valuePtr(), &mHostSysMat, mMagmaQueue);

  mMagmaOpts.solver_par.solver = Magma_PIDRMERGE;
  mMagmaOpts.solver_par.restart = 8;
  mMagmaOpts.solver_par.maxiter = 1000;
  mMagmaOpts.solver_par.rtol = 1e-10;
  mMagmaOpts.solver_par.maxiter = 1000;
  mMagmaOpts.precond_par.solver = Magma_ILU;
  mMagmaOpts.precond_par.levels = 0;
  mMagmaOpts.precond_par.trisolver = Magma_CUSOLVE;

  magma_dsolverinfo_init(&mMagmaOpts.solver_par, &mMagmaOpts.precond_par,
                         mMagmaQueue);

  magma_dvinit(&mDevRhsVec, Magma_DEV, size, 1, 0.0, mMagmaQueue);
  magma_dmtransfer(mHostSysMat, &mDevSysMat, Magma_CPU, Magma_DEV, mMagmaQueue);
  magma_d_precondsetup(mDevSysMat, mDevRhsVec, &mMagmaOpts.solver_par,
                       &mMagmaOpts.precond_par, mMagmaQueue);

  cusolverSpDestroy(mCusolverhandle);
  cusparseDestroyMatDescr(descr_M);
}

GpuMagmaAdapter::GpuMagmaAdapter() {
  magma_init();
  magma_queue_create(0, &mMagmaQueue);
  mHostSysMat = {Magma_CSR};
  mDevSysMat = {Magma_CSR};
  mHostRhsVec = {Magma_CSR};
  mDevRhsVec = {Magma_CSR};
  mHostLhsVec = {Magma_CSR};
  mDevLhsVec = {Magma_CSR};
}

void GpuMagmaAdapter::preprocessing(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  /* No preprocessing phase available yet */
}

void GpuMagmaAdapter::factorize(SparseMatrix &systemMatrix) {
  performFactorization(systemMatrix);
}

void GpuMagmaAdapter::refactorize(SparseMatrix &systemMatrix) {
  performFactorization(systemMatrix);
}

void GpuMagmaAdapter::partialRefactorize(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  performFactorization(systemMatrix);
}

Matrix GpuMagmaAdapter::solve(Matrix &mRightHandSideVector) {
  Matrix leftSideVector = mRightHandSideVector;
  int size = mRightHandSideVector.rows();
  int one = 0;

  mRightHandSideVector = *mTransp * mRightHandSideVector;

  //Copy right vector to device
  magma_dvset(size, 1, mRightHandSideVector.data(), &mHostRhsVec, mMagmaQueue);
  magma_dmtransfer(mHostRhsVec, &mDevRhsVec, Magma_CPU, Magma_DEV, mMagmaQueue);
  magma_dvinit(&mDevLhsVec, Magma_DEV, mHostRhsVec.num_rows,
               mHostRhsVec.num_cols, 0.0, mMagmaQueue);

  // Solve
  magma_d_solver(mDevSysMat, mDevRhsVec, &mDevLhsVec, &mMagmaOpts, mMagmaQueue);

  //Copy Solution back
  magma_dmtransfer(mDevLhsVec, &mHostLhsVec, Magma_DEV, Magma_CPU, mMagmaQueue);
  magma_dvcopy(mDevLhsVec, &size, &one, leftSideVector.data(), mMagmaQueue);

  return leftSideVector;
}
} // namespace DPsim
