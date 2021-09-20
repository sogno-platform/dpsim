/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverGpuDense.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverGpuDense<VarType>::MnaSolverGpuDense(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolverEigenDense<VarType>(name, domain, logLevel),
    mCusolverHandle(nullptr), mStream(nullptr) {

    mDeviceCopy = {};

    cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
    cudaError_t error = cudaSuccess;
    if((status = cusolverDnCreate(&mCusolverHandle)) != CUSOLVER_STATUS_SUCCESS)
        std::cerr << "cusolverDnCreate() failed (initializing cusolver-library)" << std::endl;
    if((error = cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking)) != cudaSuccess)
        std::cerr << cudaGetErrorString(error) << std::endl;
    if((status = cusolverDnSetStream(mCusolverHandle, mStream)) != CUSOLVER_STATUS_SUCCESS)
        std::cerr << "cusolverDnSetStream() failed" << std::endl;
}

template <typename VarType>
MnaSolverGpuDense<VarType>::~MnaSolverGpuDense() {
    //Handle & Stream
    if(mCusolverHandle)
        cusolverDnDestroy(mCusolverHandle);
    if(mStream)
        cudaStreamDestroy(mStream);

    //Memory allocated on device
    cudaFree(mDeviceCopy.matrix);
    cudaFree(mDeviceCopy.vector);
    cudaFree(mDeviceCopy.workSpace);
    cudaFree(mDeviceCopy.pivSeq);
    cudaFree(mDeviceCopy.errInfo);

    cudaDeviceReset();
}

template <typename VarType>
void MnaSolverGpuDense<VarType>::initialize() {
    MnaSolver<VarType>::initialize();

    mDeviceCopy.size = this->mRightSideVector.rows();

    //Allocate Memory on Device
    allocateDeviceMemory();
    //Copy Systemmatrix to device
    copySystemMatrixToDevice();

    // Debug logging, whether LU-factorization and copying was successfull
    /*DPsim::Matrix mat;
    mat.resize(mDeviceCopy.size, mDeviceCopy.size);
    double *buffer = &mat(0);
    CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real), cudaMemcpyDeviceToHost))
    this->mSLog->info("Systemmatrix Gpu: \n{}", mat);*/

    //LU factorization
    LUfactorization();
    /*CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real), cudaMemcpyDeviceToHost))
    this->mSLog->info("LU decomposition Gpu: \n{}", mat);*/
}

template <typename VarType>
void MnaSolverGpuDense<VarType>::allocateDeviceMemory() {
    //Allocate memory for...
    //Vector
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.vector, mDeviceCopy.size * sizeof(Real)))
    //Matrix
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.matrix, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real)))
    //Pivoting-Sequence
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.pivSeq, mDeviceCopy.size * sizeof(Real)))
    //Errorcode
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.errInfo, sizeof(int)))

    //Workspace
    int workSpaceSize = 0;
    cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
    if((status =
        cusolverDnDgetrf_bufferSize(
        mCusolverHandle,
        mDeviceCopy.size,
        mDeviceCopy.size,
        mDeviceCopy.matrix,
        mDeviceCopy.size,
        &workSpaceSize)
        ) != CUSOLVER_STATUS_SUCCESS)
        std::cerr << "cusolverDnDgetrf_bufferSize() failed (calculating required space for LU-factorization)" << std::endl;
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.workSpace, workSpaceSize))
}

template <typename VarType>
void MnaSolverGpuDense<VarType>::copySystemMatrixToDevice() {
	auto *data = MnaSolverEigenDense<VarType>::mSwitchedMatrices[MnaSolverEigenDense<VarType>::mCurrentSwitchStatus].data();
    CUDA_ERROR_HANDLER(cudaMemcpy(mDeviceCopy.matrix, data, mDeviceCopy.size * mDeviceCopy.size * sizeof(Real), cudaMemcpyHostToDevice))
}

template <typename VarType>
void MnaSolverGpuDense<VarType>::LUfactorization() {
    //Variables for error-handling
    cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
    int info;

    //LU-factorization
    status = cusolverDnDgetrf(
        mCusolverHandle,
        mDeviceCopy.size,
        mDeviceCopy.size,
        mDeviceCopy.matrix,
        mDeviceCopy.size,
        mDeviceCopy.workSpace,
        mDeviceCopy.pivSeq,
        mDeviceCopy.errInfo);

    CUDA_ERROR_HANDLER(cudaDeviceSynchronize())

    if(status != CUSOLVER_STATUS_SUCCESS) {
        std::cerr << "cusolverDnDgetrf() failed (calculating LU-factorization)" << std::endl;
    }
    CUDA_ERROR_HANDLER(cudaMemcpy(&info, mDeviceCopy.errInfo, sizeof(int), cudaMemcpyDeviceToHost))
    if(0 > info) {
        std::cerr << -info << "-th parameter is wrong" << std::endl;
    }
}

template <typename VarType>
Task::List MnaSolverGpuDense<VarType>::getTasks() {
    Task::List l;

    for (auto comp : this->mMNAComponents) {
		for (auto task : comp->mnaTasks()) {
			l.push_back(task);
		}
	}
	for (auto node : this->mNodes) {
		for (auto task : node->mnaTasks())
			l.push_back(task);
	}
	// TODO signal components should be moved out of MNA solver
	for (auto comp : this->mSimSignalComps) {
		for (auto task : comp->getTasks()) {
			l.push_back(task);
		}
	}
	l.push_back(std::make_shared<MnaSolverGpuDense<VarType>::SolveTask>(*this));
    l.push_back(std::make_shared<MnaSolverGpuDense<VarType>::LogTask>(*this));
	return l;
}

template <typename VarType>
void MnaSolverGpuDense<VarType>::solve(Real time, Int timeStepCount) {
    // Reset source vector
	this->mRightSideVector.setZero();

    // Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (const auto &stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

    if (!this->mIsInInitialization)
		this->updateSwitchStatus();

    //Copy right vector to device
    CUDA_ERROR_HANDLER(cudaMemcpy(mDeviceCopy.vector, &this->mRightSideVector(0), mDeviceCopy.size * sizeof(Real), cudaMemcpyHostToDevice))

    // Solve
	if (this->mSwitchedMatrices.size() > 0) {
        cusolverStatus_t status = cusolverDnDgetrs(
            mCusolverHandle,
            CUBLAS_OP_N,
            mDeviceCopy.size,
            1, /* nrhs */
            mDeviceCopy.matrix,
            mDeviceCopy.size,
            mDeviceCopy.pivSeq,
            mDeviceCopy.vector,
            mDeviceCopy.size,
            mDeviceCopy.errInfo);

        CUDA_ERROR_HANDLER(cudaDeviceSynchronize())

        if(status != CUSOLVER_STATUS_SUCCESS)
            std::cerr << "cusolverDnDgetrs() failed (Solving A*x = b)" << std::endl;
        int info;
        CUDA_ERROR_HANDLER(cudaMemcpy(&info, mDeviceCopy.errInfo, sizeof(int), cudaMemcpyDeviceToHost))
        if(0 > info) {
            std::cerr << -info << "-th parameter is wrong" << std::endl;
        }
    }

    //Copy Solution back
    CUDA_ERROR_HANDLER(cudaMemcpy(&this->mLeftSideVector(0), mDeviceCopy.vector, mDeviceCopy.size * sizeof(Real), cudaMemcpyDeviceToHost))

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
		this->mNodes[nodeIdx]->mnaUpdateVoltage(this->mLeftSideVector);

	// Components' states will be updated by the post-step tasks
}

}

template class DPsim::MnaSolverGpuDense<Real>;
template class DPsim::MnaSolverGpuDense<Complex>;
