/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/GpuSparseAdapter.h>

using namespace DPsim;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace DPsim {
GpuSparseAdapter::GpuSparseAdapter() {}

GpuSparseAdapter::~GpuSparseAdapter() {
  if (mCusparsehandle != nullptr) {
    cusparseDestroyMatDescr(descr_L);
    cusparseDestroyMatDescr(descr_U);
    cusparseDestroyCsrsv2Info(info_L);
    cusparseDestroyCsrsv2Info(info_U);
    cusparseDestroy(mCusparsehandle);
  }
  if (mCusolverhandle != nullptr) {
    cusolverSpDestroy(mCusolverhandle);
  }
}

inline void GpuSparseAdapter::checkCusparseStatus(cusparseStatus_t status,
                                                  std::string additionalInfo) {
  if (status != CUSPARSE_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "{} {}", additionalInfo, cusparseGetErrorString(status));
    std::cout << "Status not success: " << status << std::endl;
    throw SolverException();
  }
}

void GpuSparseAdapter::performFactorization(SparseMatrix &systemMatrix) {
  cusparseStatus_t csp_status;
  cusolverStatus_t cso_status;
  csp_status = cusparseCreate(&mCusparsehandle);
  checkCusparseStatus(csp_status, "cuSparse initialization failed:");

  if ((cso_status = cusolverSpCreate(&mCusolverhandle)) !=
      CUSOLVER_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "cuSolver initialization failed: Error code {}", cso_status);
    std::cout << "cso_status not success" << std::endl;
    throw SolverException();
  }

  size_t N = systemMatrix.rows();
  auto hMat = systemMatrix;

  mGpuRhsVec = cuda::Vector<double>(N);
  mGpuLhsVec = cuda::Vector<double>(N);
  mGpuIntermediateVec = cuda::Vector<double>(N);

  cusparseMatDescr_t descr_M = 0;
  csrilu02Info_t info_M = 0;
  int structural_zero;
  int numerical_zero;

  size_t nnz = hMat.nonZeros();

  // step 1: create a descriptor which contains
  // - matrix M is base-1
  // - matrix L is base-1
  // - matrix L is lower triangular
  // - matrix L has unit diagonal
  // - matrix U is base-1
  // - matrix U is upper triangular
  // - matrix U has non-unit diagonal
  checkCusparseStatus(cusparseCreateMatDescr(&descr_M));
  checkCusparseStatus(
      cusparseSetMatIndexBase(descr_M, CUSPARSE_INDEX_BASE_ZERO));
  checkCusparseStatus(
      cusparseSetMatType(descr_M, CUSPARSE_MATRIX_TYPE_GENERAL));

  checkCusparseStatus(cusparseCreateMatDescr(&descr_L));
  checkCusparseStatus(
      cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ZERO));
  checkCusparseStatus(
      cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL));
  checkCusparseStatus(
      cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER));
  checkCusparseStatus(cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_UNIT));

  checkCusparseStatus(cusparseCreateMatDescr(&descr_U));
  checkCusparseStatus(
      cusparseSetMatIndexBase(descr_U, CUSPARSE_INDEX_BASE_ZERO));
  checkCusparseStatus(
      cusparseSetMatType(descr_U, CUSPARSE_MATRIX_TYPE_GENERAL));
  checkCusparseStatus(
      cusparseSetMatFillMode(descr_U, CUSPARSE_FILL_MODE_UPPER));
  checkCusparseStatus(
      cusparseSetMatDiagType(descr_U, CUSPARSE_DIAG_TYPE_NON_UNIT));

  // step 2: create a empty info structure
  // we need one info for csrilu02 and two info's for csrsv2
  checkCusparseStatus(cusparseCreateCsrilu02Info(&info_M));
  checkCusparseStatus(cusparseCreateCsrsv2Info(&info_L));
  checkCusparseStatus(cusparseCreateCsrsv2Info(&info_U));

  // step 2a: permutate Matrix M' = P*M
  int p_nnz = 0;
  int p[N];

  cso_status = cusolverSpDcsrzfdHost(mCusolverhandle, N, nnz, descr_M,
                                     hMat.valuePtr(), hMat.outerIndexPtr(),
                                     hMat.innerIndexPtr(), p, &p_nnz);

  if (cso_status != CUSOLVER_STATUS_SUCCESS) {
    //SPDLOG_LOGGER_ERROR(mSLog, "cusolverSpDcsrzfdHost returend an error");
  }
  // create Eigen::PermutationMatrix from the p
  mTransp = std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic>>(
      new Eigen::PermutationMatrix<Eigen::Dynamic>(
          Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, 1>>(p, N, 1)));

  // apply permutation
  hMat = *mTransp * hMat;

  // copy P' to GPU
  mSysMat = std::unique_ptr<cuda::CudaMatrix<double, int>>(
      new cuda::CudaMatrix<double, int>(hMat, N));

  double *d_csrVal = mSysMat->val.data();
  int *d_csrRowPtr = mSysMat->row.data();
  int *d_csrColInd = mSysMat->col.data();

  // step 3: query how much memory used in csrilu02 and csrsv2, and allocate the buffer
  int pBufferSize_M, pBufferSize_L, pBufferSize_U;
  int pBufferSize;
  csp_status = cusparseDcsrilu02_bufferSize(mCusparsehandle, N, nnz, descr_M,
                                            d_csrVal, d_csrRowPtr, d_csrColInd,
                                            info_M, &pBufferSize_M);
  checkCusparseStatus(csp_status, "failed to get cusparse bufferSize:");

  csp_status = cusparseDcsrsv2_bufferSize(
      mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L,
      d_csrVal, d_csrRowPtr, d_csrColInd, info_L, &pBufferSize_L);
  checkCusparseStatus(csp_status, "failed to get cusparse bufferSize:");

  csp_status = cusparseDcsrsv2_bufferSize(
      mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_U,
      d_csrVal, d_csrRowPtr, d_csrColInd, info_U, &pBufferSize_U);
  checkCusparseStatus(csp_status, "failed to get cusparse bufferSize:");

  // Buffer
  pBufferSize = std::max({pBufferSize_M, pBufferSize_L, pBufferSize_U});
  pBuffer = cuda::Vector<char>(pBufferSize);
  // step 4: perform analysis of incomplete Cholesky on M
  //         perform analysis of triangular solve on L
  //         perform analysis of triangular solve on U
  // The lower(upper) triangular part of M has the same sparsity pattern as L(U),
  // we can do analysis of csrilu0 and csrsv2 simultaneously.
  csp_status = cusparseDcsrilu02_analysis(
      mCusparsehandle, N, nnz, descr_M, d_csrVal, d_csrRowPtr, d_csrColInd,
      info_M, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer.data());
  checkCusparseStatus(csp_status, "failed to analyse cusparse problem:");

  csp_status =
      cusparseXcsrilu02_zeroPivot(mCusparsehandle, info_M, &structural_zero);
  if (csp_status == CUSPARSE_STATUS_ZERO_PIVOT) {
    //SPDLOG_LOGGER_ERROR(mSLog, "A({},{}) is missing", structural_zero, structural_zero);
    checkCusparseStatus(csp_status);
    std::cout << "csp_status zero pivot" << std::endl;
    throw SolverException();
  }

  csp_status = cusparseDcsrsv2_analysis(
      mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L,
      d_csrVal, d_csrRowPtr, d_csrColInd, info_L,
      CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer.data());
  checkCusparseStatus(csp_status, "failed to analyse cusparse problem:");

  csp_status = cusparseDcsrsv2_analysis(
      mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_U,
      d_csrVal, d_csrRowPtr, d_csrColInd, info_U,
      CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer.data());
  checkCusparseStatus(csp_status, "failed to analyse cusparse problem:");

  // step 5: M = L * U

  csp_status = cusparseDcsrilu02(
      mCusparsehandle, N, nnz, descr_M, d_csrVal, d_csrRowPtr, d_csrColInd,
      info_M, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer.data());
  checkCusparseStatus(csp_status, "failed to perform cusparse ILU:");

  csp_status =
      cusparseXcsrilu02_zeroPivot(mCusparsehandle, info_M, &numerical_zero);
  if (csp_status == CUSPARSE_STATUS_ZERO_PIVOT) {
    //SPDLOG_LOGGER_ERROR(mSLog, "U({},{}) is zero\n", numerical_zero, numerical_zero);
    checkCusparseStatus(csp_status);
    std::cout << "csp_status zero pivot" << std::endl;
    throw SolverException();
  }

  //std::cout << *mSysMat.get() << std::endl;

  csp_status = cusparseDestroyCsrilu02Info(info_M);
  checkCusparseStatus(csp_status, "failed to destroy info:");
  csp_status = cusparseDestroyMatDescr(descr_M);
  checkCusparseStatus(csp_status, "failed to destroy MatDescr:");
}

void GpuSparseAdapter::preprocessing(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  /* No preprocessing phase available yet */
}

void GpuSparseAdapter::factorize(SparseMatrix &systemMatrix) {
  performFactorization(systemMatrix);
}

void GpuSparseAdapter::refactorize(SparseMatrix &systemMatrix) {
  performFactorization(systemMatrix);
}

void GpuSparseAdapter::partialRefactorize(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  performFactorization(systemMatrix);
}

Matrix GpuSparseAdapter::solve(Matrix &mRightHandSideVector) {
  Matrix leftSideVector = mRightHandSideVector;
  cudaError_t status;
  cusparseStatus_t csp_status;
  int size = mRightHandSideVector.rows();

  //Copy right vector to device
  //Permutate right side: R' = P * R
  mRightHandSideVector = *mTransp * mRightHandSideVector;
  status = cudaMemcpy(mGpuRhsVec.data(), &mRightHandSideVector(0),
                      size * sizeof(Real), cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    //SPDLOG_LOGGER_ERROR(mSLog, "Cuda Error: {}", cudaGetErrorString(status));
    std::cout << "status not cudasuccess" << std::endl;
    throw SolverException();
  }

  const double alpha = 1.;
  // Solve
  // step 6: solve L*z = x
  csp_status = cusparseDcsrsv2_solve(
      mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, size,
      mSysMat->non_zero, &alpha, descr_L, mSysMat->val.data(),
      mSysMat->row.data(), mSysMat->col.data(), info_L, mGpuRhsVec.data(),
      mGpuIntermediateVec.data(), CUSPARSE_SOLVE_POLICY_NO_LEVEL,
      pBuffer.data());
  checkCusparseStatus(csp_status, "failed to solve L*z=x:");

  // step 7: solve U*y = z
  csp_status = cusparseDcsrsv2_solve(
      mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, size,
      mSysMat->non_zero, &alpha, descr_U, mSysMat->val.data(),
      mSysMat->row.data(), mSysMat->col.data(), info_U,
      mGpuIntermediateVec.data(), mGpuLhsVec.data(),
      CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer.data());
  checkCusparseStatus(csp_status, "failed to solve U*y=z:");

  //Copy Solution back
  status = cudaMemcpy(&(leftSideVector)(0), mGpuLhsVec.data(),
                      size * sizeof(Real), cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    //SPDLOG_LOGGER_ERROR(mSLog, "Cuda Error: {}", cudaGetErrorString(status));
    std::cout << "status not cudasuccess" << std::endl;
    throw SolverException();
  }
  return leftSideVector;
}
} // namespace DPsim
