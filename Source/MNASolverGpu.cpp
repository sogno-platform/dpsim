#include <dpsim/MNASolverGpu.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverGpu<VarType>::MnaSolverGpu(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolver<VarType>(name, domain, logLevel),
    mCusolverHandle(nullptr), mStream(nullptr) {

    mDeviceCopy = {};

    cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
    cudaError_t error = cudaSuccess;
    if((status = cusolverDnCreate(&mCusolverHandle)) != CUSOLVER_STATUS_SUCCESS)
        std::cerr << "cusolverDnCreate() failed" << std::endl;
    if((error = cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking)) != cudaSuccess)
        std::cerr << cudaGetErrorString(error) << std::endl;
    if((status = cusolverDnSetStream(mCusolverHandle, mStream)) != CUSOLVER_STATUS_SUCCESS)
        std::cerr << "cusolverDnSetStream() failed" << std::endl;
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

    //Allocate Memory on Device
    allocateDeviceMemory();
    //Copy Systemmatrix to device
    copySystemMatrixToDevice();
    
    auto index = this->mRightSideVector.rows();
    DPsim::Matrix mat;
    mat.resize(index, index);
    double *buffer = &mat(0);
    CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, index * index * sizeof(Real), cudaMemcpyDeviceToHost))
    this->mSLog->info("Systemmatrix Gpu: \n{}", mat);

    //LU factorization
    LUfactorization();
    CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mDeviceCopy.matrix, index * index * sizeof(Real), cudaMemcpyDeviceToHost))
    this->mSLog->info("LU decomposition Gpu: \n{}", mat);
}

/// Allocate Space for Vectors & Matrices
template <typename VarType>
void MnaSolverGpu<VarType>::allocateDeviceMemory() {
    //Get required size
    auto index = this->mRightSideVector.rows();
    auto size = index * sizeof(Real);
    
    //Vectors
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.rightVector, size))
    //Matrix
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.matrix, size * index))
    //Pivoting-Sequence
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.pivSeq, size))
    //Errorcode
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.errInfo, sizeof(int)))

    //Workspace
    int workSpaceSize = 0;
    cusolverStatus_t status = CUSOLVER_STATUS_SUCCESS;
    if((status = 
        cusolverDnDgetrf_bufferSize(
        mCusolverHandle,
        index,
        index,
        mDeviceCopy.matrix,
        index,
        &workSpaceSize)
        ) != CUSOLVER_STATUS_SUCCESS)
        std::cerr << "cusolverDnDgetrf_bufferSize() failed" << std::endl;
    CUDA_ERROR_HANDLER(cudaMalloc((void**)&mDeviceCopy.workSpace, workSpaceSize))
}

template <typename VarType>
void MnaSolverGpu<VarType>::copySystemMatrixToDevice() {
    //TODO Error Checking
    auto dim = this->mRightSideVector.rows();
    Real *mat = &MnaSolver<VarType>::systemMatrix()(0);
    CUDA_ERROR_HANDLER(cudaMemcpy(mDeviceCopy.matrix, mat, sizeof(Real) * dim * dim, cudaMemcpyHostToDevice))
}

template <typename VarType>
void MnaSolverGpu<VarType>::LUfactorization() {
    auto dim = this->mRightSideVector.rows();
    //TODO Error checking
    cusolverStatus_t status;
    status = cusolverDnDgetrf(
        mCusolverHandle,
        dim,
        dim,
        mDeviceCopy.matrix,
        dim,
        mDeviceCopy.workSpace,
        mDeviceCopy.pivSeq,
        mDeviceCopy.errInfo);
    if(status != CUSOLVER_STATUS_SUCCESS) {
        std::cerr << "cusolverDnDgetrf() failed" << std::endl;
    }
    int info;
    CUDA_ERROR_HANDLER(cudaMemcpy(&info, mDeviceCopy.errInfo, sizeof(int), cudaMemcpyDeviceToHost))
    if(0 > info) {
        std::cerr << -info << "-th parameter is wrong" << std::endl;
    }
    CUDA_ERROR_HANDLER(cudaDeviceSynchronize())
}

template <typename VarType>
Task::List MnaSolverGpu<VarType>::getTasks() {
    Task::List l;

	for (const auto &comp : this->mPowerComponents) {
		for (auto task : comp->mnaTasks()) {
			l.push_back(task);
		}
	}
	for (const auto &node : this->mNodes) {
		for (auto task : node->mnaTasks())
			l.push_back(task);
	}
	// TODO signal components should be moved out of MNA solver
	for (const auto &comp : this->mSignalComponents) {
		for (auto task : comp->getTasks()) {
			l.push_back(task);
		}
	}
	l.push_back(std::make_shared<MnaSolverGpu<VarType>::SolveTask>(*this, false));
    l.push_back(std::make_shared<MnaSolverGpu<VarType>::LogTask>(*this));
	return l;
}

template <typename VarType>
void MnaSolverGpu<VarType>::SolveTask::execute(Real time, Int timeStepCount) {
    const auto dim = mSolver.mRightSideVector.rows();
    const auto size = dim * sizeof(Real);
    // Reset source vector
	mSolver.mRightSideVector.setZero();

    // Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (const auto &stamp : mSolver.mRightVectorStamps)
		mSolver.mRightSideVector += *stamp;

    //Copy right vector to device
    CUDA_ERROR_HANDLER(cudaMemcpy(mSolver.mDeviceCopy.rightVector, &mSolver.mRightSideVector(0), size, cudaMemcpyHostToDevice))
    mSolver.mSLog->info("Right-Side-Vector Cpu: \n{}", mSolver.mRightSideVector);

    //Print RHS-vector
    DPsim::Matrix mat;
    mat.resize(dim, 1);
    double *buffer = &mat(0);
    CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mSolver.mDeviceCopy.rightVector, dim * sizeof(Real), cudaMemcpyDeviceToHost))
    mSolver.mSLog->info("Right-Side-Vector Gpu: \n{}", mat);

    // Solve
	if (mSolver.mSwitchedMatrices.size() > 0) {
        cusolverStatus_t status = cusolverDnDgetrs(
            mSolver.mCusolverHandle,
            CUBLAS_OP_N,
            dim,
            1, /* nrhs */
            mSolver.mDeviceCopy.matrix,
            dim,
            mSolver.mDeviceCopy.pivSeq,
            mSolver.mDeviceCopy.rightVector,
            dim,
            mSolver.mDeviceCopy.errInfo);

        if(status != CUSOLVER_STATUS_SUCCESS)
            std::cerr << "cusolverDnDgetrs() failed" << std::endl;
        int info;
        CUDA_ERROR_HANDLER(cudaMemcpy(&info, mSolver.mDeviceCopy.errInfo, sizeof(int), cudaMemcpyDeviceToHost))
        if(0 > info) {
            std::cerr << -info << "-th parameter is wrong" << std::endl;
        }
    }

    //Copy Leftvector back
    buffer = new double[size];
    CUDA_ERROR_HANDLER(cudaMemcpy(buffer, mSolver.mDeviceCopy.rightVector, size, cudaMemcpyDeviceToHost))
    for(UInt i = 0; i < dim; i++) {
        mSolver.mLeftSideVector(i, 0) = buffer[i]; // TODO check
    }

    delete[] buffer;

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < mSolver.mNumNetNodes; nodeIdx++)
		mSolver.mNodes[nodeIdx]->mnaUpdateVoltage(mSolver.mLeftSideVector);

	if (!mSteadyStateInit)
		mSolver.updateSwitchStatus();

	// Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverGpu<VarType>::LogTask::execute(Real time, Int timeStepCount) {
	mSolver.log(time);
}

}

template class DPsim::MnaSolverGpu<Real>;
template class DPsim::MnaSolverGpu<Complex>;