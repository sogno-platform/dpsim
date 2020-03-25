#include <dpsim/MNASolverGpu.h>
#include <dpsim/Definitions.h>

using namespace DPsim;

namespace DPsim {

template <typename VarType>
MnaSolverGpu<VarType>::MnaSolverGpu() :
mCusolverHandle(nullptr), mStream(nullptr),
mGpuSystemMatrix(nullptr), mGpuLeftVector(nullptr), mGpuRightVector(nullptr) {

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

    //Matrix & Vectors
    if(mGpuSystemMatrix)
        cudaFree(mGpuSystemMatrix);
    if(mGpuLeftVector)
        cudaFree(mGpuLeftVector);
    if(mGpuRightVector)
        cudaFree(mGpuRightVector);

    cudaDeviceReset();
}

template <typename VarType>
void MnaSolverGpu<VarType>::initialize() {
    MnaSolver::initialize();

    createEmptyVectors();
    createEmptySystemMatrix();

    //Copy Systemmatrix to device
}

/// Allocate Space for Vectors & Matrices
template <typename VarType>
void MnaSolverGpu<VarType>::createEmptyVectors() {
    //TODO Error Checking
    auto size = sizeof(Real) * mNumSimNodes;
    cudaError_t stat;
    stat = cudaMalloc(static_cast<void**>(&mGpuLeftVector), size);
    stat = cudaMalloc(static_cast<void**>(&mGpuRightVector), size);
}

template <typename VarType>
void MnaSolverGpu<VarType>::createEmptySystemMatrix() {
    //TODO Error Checking
    auto size = sizeof(Real) * mNumSimNodes * mNumSimNodes;
    cudaError_t stat = cudaMalloc(static_cast<void**>(&mGpuLeftVector), size);
}

}

template class DPsim::MnaSolverGpu<Real>;
template class DPsim::MnaSolverGpu<Complex>;