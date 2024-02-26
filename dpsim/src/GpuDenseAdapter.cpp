/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/GpuDenseAdapter.h>

using namespace DPsim;

namespace DPsim {
GpuDenseAdapter::~GpuDenseAdapter() {
  //Handle & Stream
  if (mCusolverHandle)
    cusolverDnDestroy(mCusolverHandle);
  if (mStream)
    cudaStreamDestroy(mStream);

  //Memory allocated on device
  cudaFree(mDeviceCopy.matrix);
  cudaFree(mDeviceCopy.vector);
  cudaFree(mDeviceCopy.workSpace);
  cudaFree(mDeviceCopy.pivSeq);
  cudaFree(mDeviceCopy.errInfo);

  cudaDeviceReset();
}

GpuDenseAdapter::GpuDenseAdapter() {
  mDeviceCopy = {};

  cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
  cudaError_t error = cudaSuccess;
  if ((status = cusolverDnCreate(&mCusolverHandle)) != CUSOLVER_STATUS_SUCCESS)
    std::cerr << "cusolverDnCreate() failed (initializing cusolver-library)"
              << std::endl;
  if ((error = cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking)) !=
      cudaSuccess)
    std::cerr << cudaGetErrorString(error) << std::endl;
  if ((status = cusolverDnSetStream(mCusolverHandle, mStream)) !=
      CUSOLVER_STATUS_SUCCESS)
    std::cerr << "cusolverDnSetStream() failed" << std::endl;
}

void GpuDenseAdapter::allocateDeviceMemory() {
  //Allocate memory for...
  //Vector
  CUDA_ERROR_HANDLER(
      cudaMalloc((void **)&mDeviceCopy.vector, mDeviceCopy.size * sizeof(Real)))
  //Matrix
  CUDA_ERROR_HANDLER(
      cudaMalloc((void **)&mDeviceCopy.matrix,
                 mDeviceCopy.size * mDeviceCopy.size * sizeof(Real)))
  //Pivoting-Sequence
  CUDA_ERROR_HANDLER(
      cudaMalloc((void **)&mDeviceCopy.pivSeq, mDeviceCopy.size * sizeof(Real)))
  //Errorcode
  CUDA_ERROR_HANDLER(cudaMalloc((void **)&mDeviceCopy.errInfo, sizeof(int)))

  //Workspace
  int workSpaceSize = 0;
  cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
  if ((status = cusolverDnDgetrf_bufferSize(
           mCusolverHandle, mDeviceCopy.size, mDeviceCopy.size,
           mDeviceCopy.matrix, mDeviceCopy.size, &workSpaceSize)) !=
      CUSOLVER_STATUS_SUCCESS)
    std::cerr << "cusolverDnDgetrf_bufferSize() failed (calculating required "
                 "space for LU-factorization)"
              << std::endl;
  CUDA_ERROR_HANDLER(cudaMalloc((void **)&mDeviceCopy.workSpace, workSpaceSize))
}

void GpuDenseAdapter::copySystemMatrixToDevice(Matrix systemMatrix) {
  auto *data = systemMatrix.data();
  CUDA_ERROR_HANDLER(
      cudaMemcpy(mDeviceCopy.matrix, data,
                 mDeviceCopy.size * mDeviceCopy.size * sizeof(Real),
                 cudaMemcpyHostToDevice))
}

void GpuDenseAdapter::LUfactorization() {
  //Variables for error-handling
  cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
  int info;

  //LU-factorization
  status = cusolverDnDgetrf(mCusolverHandle, mDeviceCopy.size, mDeviceCopy.size,
                            mDeviceCopy.matrix, mDeviceCopy.size,
                            mDeviceCopy.workSpace, mDeviceCopy.pivSeq,
                            mDeviceCopy.errInfo);

  CUDA_ERROR_HANDLER(cudaDeviceSynchronize())

  if (status != CUSOLVER_STATUS_SUCCESS) {
    std::cerr << "cusolverDnDgetrf() failed (calculating LU-factorization)"
              << std::endl;
  }
  CUDA_ERROR_HANDLER(cudaMemcpy(&info, mDeviceCopy.errInfo, sizeof(int),
                                cudaMemcpyDeviceToHost))
  if (0 > info) {
    std::cerr << -info << "-th parameter is wrong" << std::endl;
  }
}

void GpuDenseAdapter::preprocessing(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  mDeviceCopy.size = systemMatrix.rows();

  //Allocate Memory on Device
  allocateDeviceMemory();
  //Copy Systemmatrix to device
  copySystemMatrixToDevice(Matrix(systemMatrix));

  // Debug logging, whether LU-factorization and copying was successfull
  /*DPsim::Matrix mat;
        mat.resize(mDeviceCopy.size, mDeviceCopy.size);
        double *buffer = &mat(0);
        CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real), cudaMemcpyDeviceToHost))
        this->SPDLOG_LOGGER_INFO(mSLog, "Systemmatrix Gpu: \n{}", mat);*/
}

void GpuDenseAdapter::factorize(SparseMatrix &systemMatrix) {
  //Copy Systemmatrix to device
  copySystemMatrixToDevice(Matrix(systemMatrix));
  //LU factorization
  LUfactorization();
  /*CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real), cudaMemcpyDeviceToHost))
        this->SPDLOG_LOGGER_INFO(mSLog, "LU decomposition Gpu: \n{}", mat);*/
}

void GpuDenseAdapter::refactorize(SparseMatrix &systemMatrix) {
  //Copy Systemmatrix to device
  copySystemMatrixToDevice(Matrix(systemMatrix));
  //LU factorization
  LUfactorization();
  /*CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real), cudaMemcpyDeviceToHost))
        this->SPDLOG_LOGGER_INFO(mSLog, "LU decomposition Gpu: \n{}", mat);*/
}

void GpuDenseAdapter::partialRefactorize(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  //Copy Systemmatrix to device
  copySystemMatrixToDevice(Matrix(systemMatrix));
  //LU factorization
  LUfactorization();
  /*CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real), cudaMemcpyDeviceToHost))
        this->SPDLOG_LOGGER_INFO(mSLog, "LU decomposition Gpu: \n{}", mat);*/
}

Matrix GpuDenseAdapter::solve(Matrix &mRightHandSideVector) {
  Matrix leftSideVector = mRightHandSideVector;

  CUDA_ERROR_HANDLER(cudaMemcpy(mDeviceCopy.vector, &mRightHandSideVector(0),
                                mDeviceCopy.size * sizeof(Real),
                                cudaMemcpyHostToDevice))

  cusolverStatus_t status = cusolverDnDgetrs(
      mCusolverHandle, CUBLAS_OP_N, mDeviceCopy.size, 1, /* nrhs */
      mDeviceCopy.matrix, mDeviceCopy.size, mDeviceCopy.pivSeq,
      mDeviceCopy.vector, mDeviceCopy.size, mDeviceCopy.errInfo);

  CUDA_ERROR_HANDLER(cudaDeviceSynchronize())

  if (status != CUSOLVER_STATUS_SUCCESS)
    std::cerr << "cusolverDnDgetrs() failed (Solving A*x = b)" << std::endl;
  int info;
  CUDA_ERROR_HANDLER(cudaMemcpy(&info, mDeviceCopy.errInfo, sizeof(int),
                                cudaMemcpyDeviceToHost))
  if (0 > info) {
    std::cerr << -info << "-th parameter is wrong" << std::endl;
  }

  CUDA_ERROR_HANDLER(cudaMemcpy(&(leftSideVector)(0), mDeviceCopy.vector,
                                mDeviceCopy.size * sizeof(Real),
                                cudaMemcpyDeviceToHost))
  //return LUFactorizedSparse.solve(mRightHandSideVector);

  return leftSideVector;
}
} // namespace DPsim
