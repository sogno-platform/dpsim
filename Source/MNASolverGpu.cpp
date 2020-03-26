#include <dpsim/MNASolverGpu.h>
#include <dpsim/Definitions.h>

using namespace DPsim;

namespace DPsim {

template <typename VarType>
MnaSolverGpu<VarType>::MnaSolverGpu() :
mCusolverHandle(nullptr), mStream(nullptr) {

    mDeviceCopy = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
    cudaError_t cudaErrorCode;

    //TODO Error Checking
    status = cusolverDnCreate(&mCusolverHandle);
    cudaErrorCode = cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking);
    status = cusolverDnSetStream(cusolverH, stream);
}

template <typename VarType>
MnaSolverGpu<VarType>::~MnaSolverGpu() {
    //Handle & Stream
    if(mCusolverHandle)
        cusolverDnDestroy(mCusolverHandle);
    if(mStream)
        cudaStreamDestroy(stream);

    //Memory allocated on device
    if(mDeviceCopy.matrix)
        cudaFree(mDeviceCopy.matrix);
    if(mDeviceCopy.rightVector)
        cudaFree(mDeviceCopy.rightVector);
    if(mDeviceCopy.workSpace)
        cudaFree(mDeviceCopy.workSpace);
    if(mDeviceCopy.pivSeq)
        cudaFree(mDeviceCopy.pivSeq);
    if(mDeviceCopy.errInfo)
        cudaFree(mDevice.errInfo);

    cudaDeviceReset();
}

template <typename VarType>
void MnaSolverGpu<VarType>::initialize() {
    MnaSolver::initialize();

    allocateDeviceMemory();
    //Copy Systemmatrix to device
    copySystemMatrixToDevice();
    //LU factorization
}

/// Allocate Space for Vectors & Matrices
template <typename VarType>
void MnaSolverGpu<VarType>::allocateDeviceMemory() {
    //TODO Error checking
    //Vectors
    auto size = sizeof(Real) * mNumSimNodes;
    cudaError_t stat;
    stat = cudaMalloc(static_cast<void**>(&mDeviceCopy.leftVector), size);
    stat = cudaMalloc(static_cast<void**>(&mDeviceCopy.rightVector), size);
    //Matrix
    size = sizeof(Real) * mNumSimNodes * mNumSimNodes;
    stat = cudaMalloc(static_cast<void**>(&mDeviceCopy.matrix), size);
    //Pivoting-Sequence
    stat = cudaMalloc(static_cast<void**>(&mDeviceCopy.matrix), sizeof(int) * mNumSimNodes);
    //Errorcode
    stat = cudaMalloc(static_cast<void**>(&mDeviceCopy.errInfo), sizeof(int));

    //Workspace
    int workSpaceSize = 0;
    cusolverStatus_t status = cusolverDnDgetrf_bufferSize(
        mCusolverHandle,
        mNumSimNodes,
        mNumSimNodes,
        mDeviceCopy.matrix,
        mNumSimNodes,
        &workSpaceSize);
    stat = cudaMalloc(&mDeviceCopy.workSpace, workSpaceSize);
}

template <typename VarType>
void MnaSolverGpu<VarType>::copySystemMatrixToDevice() {
    //TODO Error Checking
    Real *mat = new Real[mNumSimNodes * mNumSimNodes];
    for(int i = 0; i < mNumSimNodes; i++) {
        for(int j = 0; j < mNumSimNodes; j++) {
            mat[i * mNumSimNodes + j] = systemMatrix()(i, j);
        }
    }
    cudaError_t cudaStat1 = cudaMemcpy(mDeviceCopy.matrix, mat, sizeof(Real) * mNumSimNodes * mNumSimNodes, cudaMemcpyHostToDevice);
}

template <typename VarType>
void MnaSolverGpu<VarType>::LUfactorization() {
    //TODO Error checking
    auto status = cusolverDnDgetrf(
        mCusolverHandle,
        mNumSimNodes,
        mNumSimNodes,
        mDeviceCopy.matrix,
        mNumSimNodes,
        mDeviceCopy.workSpace,
        mDeviceCopy.pivSeq,
        mDeviceCopy.errInfo);
    status = cudaDeviceSynchronize();
}

template <typename VarType>
void MnaSolverGpu<VarType>::SolveTask::execute(Real time, Int timeStepCount) {
    // Reset source vector
	mSolver.mRightSideVector.setZero();

    // Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (auto stamp : mSolver.mRightVectorStamps)
		mSolver.mRightSideVector += *stamp;
    //Copy to device
    double *buffer = new double[mNumSimNodes];
    for(int i = 0; i < mNumSimNodes; i++) {
        buffer[i] = mSolver.mRightSideVector(1, i); //TODO check
    }
    cudaError_t ec = cudaMemcpy(mDeviceCopy.rightVector, buffer, mNumSimNodes, cudaMemcpyHostToDevice);

    // Solve
	if (mSolver.mSwitchedMatrices.size() > 0) {
        auto status = cusolverDnDgetrs(
            mCusolverHandle,
            CUBLAS_OP_N,
            mNumSimNodes,
            1, /* nrhs */
            mDeviceCopy.matrix,
            mNumSimNodes,
            mDeviceCopy.pivSeq,
            mDeviceCopy.rightVector,
            mNumSimNodes,
            mDeviceCopy.errInfo);
    }

    //Copy Leftvector back
    ec = cudaMemcpy(buffer, mDeviceCopy.rightVector, mNumSimNodes, cudaMemcpyDeviceToHost);
    for(int i = 0; i < mNumSimNodes; i++) {
        mSolver.mLeftSideVector(1, i) = buffer[i]; // TODO check
    }

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mSolver.mNumNetNodes; nodeIdx++)
		mSolver.mNodes[nodeIdx]->mnaUpdateVoltage(mSolver.mLeftSideVector);

	if (!mSteadyStateInit)
		mSolver.updateSwitchStatus();

	// Components' states will be updated by the post-step tasks
}

}

template class DPsim::MnaSolverGpu<Real>;
template class DPsim::MnaSolverGpu<Complex>;