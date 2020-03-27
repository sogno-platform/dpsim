#include <dpsim/MNASolverGpu.h>
#include <dpsim/Definitions.h>

using namespace DPsim;

namespace DPsim {

template <typename VarType>
MnaSolverGpu<VarType>::MnaSolverGpu(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolver<VarType>(name, domain, logLevel),
    mCusolverHandle(nullptr), mStream(nullptr) {

    mDeviceCopy = {};

    //TODO Error Checking
    cusolverDnCreate(&mCusolverHandle);
    cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking);
    cusolverDnSetStream(mCusolverHandle, mStream);
}

template <typename VarType>
MnaSolverGpu<VarType>::~MnaSolverGpu() {
    //Handle & Stream
    if(mCusolverHandle)
        cusolverDnDestroy(mCusolverHandle);
    if(mStream)
        cudaStreamDestroy(mStream);

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
        cudaFree(mDeviceCopy.errInfo);

    cudaDeviceReset();
}

template <typename VarType>
void MnaSolverGpu<VarType>::initialize() {
    MnaSolver<VarType>::initialize();

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
    auto size = sizeof(Real) * this->mNumSimNodes;
    cudaMalloc((void**)&mDeviceCopy.rightVector, size);
    //Matrix
    size = sizeof(Real) * this->mNumSimNodes * this->mNumSimNodes;
    cudaMalloc((void**)&mDeviceCopy.matrix, size);
    //Pivoting-Sequence
    cudaMalloc((void**)&mDeviceCopy.matrix, sizeof(int) * this->mNumSimNodes);
    //Errorcode
    cudaMalloc((void**)&mDeviceCopy.errInfo, sizeof(int));

    //Workspace
    int workSpaceSize = 0;
    cusolverDnDgetrf_bufferSize(
        mCusolverHandle,
        this->mNumSimNodes,
        this->mNumSimNodes,
        mDeviceCopy.matrix,
        this->mNumSimNodes,
        &workSpaceSize);
    cudaMalloc((void**)&mDeviceCopy.workSpace, workSpaceSize);
}

template <typename VarType>
void MnaSolverGpu<VarType>::copySystemMatrixToDevice() {
    //TODO Error Checking
    Real *mat = new Real[this->mNumSimNodes * this->mNumSimNodes];
    for(UInt i = 0; i < this->mNumSimNodes; i++) {
        for(UInt j = 0; j < this->mNumSimNodes; j++) {
            mat[i * this->mNumSimNodes + j] = MnaSolver<VarType>::systemMatrix()(i, j);
        }
    }
    cudaMemcpy(mDeviceCopy.matrix, mat, sizeof(Real) * this->mNumSimNodes * this->mNumSimNodes, cudaMemcpyHostToDevice);
}

template <typename VarType>
void MnaSolverGpu<VarType>::LUfactorization() {
    //TODO Error checking
    cusolverDnDgetrf(
        mCusolverHandle,
        this->mNumSimNodes,
        this->mNumSimNodes,
        mDeviceCopy.matrix,
        this->mNumSimNodes,
        mDeviceCopy.workSpace,
        mDeviceCopy.pivSeq,
        mDeviceCopy.errInfo);
    cudaDeviceSynchronize();
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
    double *buffer = new double[mSolver.mNumSimNodes];
    for(UInt i = 0; i < mSolver.mNumSimNodes; i++) {
        buffer[i] = mSolver.mRightSideVector(1, i); //TODO check
    }
    //cudaError_t ec = 
    cudaMemcpy(mSolver.mDeviceCopy.rightVector, buffer, mSolver.mNumSimNodes, cudaMemcpyHostToDevice);

    // Solve
	if (mSolver.mSwitchedMatrices.size() > 0) {
        cusolverDnDgetrs(
            mSolver.mCusolverHandle,
            CUBLAS_OP_N,
            mSolver.mNumSimNodes,
            1, /* nrhs */
            mSolver.mDeviceCopy.matrix,
            mSolver.mNumSimNodes,
            mSolver.mDeviceCopy.pivSeq,
            mSolver.mDeviceCopy.rightVector,
            mSolver.mNumSimNodes,
            mSolver.mDeviceCopy.errInfo);
    }

    //Copy Leftvector back
    cudaMemcpy(buffer, mSolver.mDeviceCopy.rightVector, mSolver.mNumSimNodes, cudaMemcpyDeviceToHost);
    for(UInt i = 0; i < mSolver.mNumSimNodes; i++) {
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