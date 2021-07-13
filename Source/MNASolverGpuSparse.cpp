#include <dpsim/MNASolverGpuSparse.h>
#include <dpsim/SequentialScheduler.h>
#include <Eigen/Eigen>

using namespace DPsim;
using namespace CPS;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
namespace DPsim {

template <typename VarType>
MnaSolverGpuSparse<VarType>::MnaSolverGpuSparse(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolverEigenSparse<VarType>(name, domain, logLevel),
    mCusparsehandle(nullptr), mSysMat(nullptr),
	mTransp(nullptr),
	mGpuRhsVec(0), mGpuLhsVec(0), mGpuIntermediateVec(0),
	pBuffer(0) {

}

template <typename VarType>
MnaSolverGpuSparse<VarType>::~MnaSolverGpuSparse() {
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

template <typename VarType>
inline void MnaSolverGpuSparse<VarType>::checkCusparseStatus(cusparseStatus_t status, std::string additionalInfo)
{
	if (status != CUSPARSE_STATUS_SUCCESS) {
		mSLog->error("{} {}", additionalInfo, cusparseGetErrorString(status));
		throw SolverException();
	}
}

template <typename VarType>
void MnaSolverGpuSparse<VarType>::initialize() {
    MnaSolver<VarType>::initialize();

	cusparseStatus_t csp_status;
	cusolverStatus_t cso_status;
	csp_status = cusparseCreate(&mCusparsehandle);
	checkCusparseStatus(csp_status, "cuSparse initialization failed:");

	if ((cso_status = cusolverSpCreate(&mCusolverhandle)) != CUSOLVER_STATUS_SUCCESS) {
		mSLog->error("cuSolver initialization failed: Error code {}", cso_status);
		throw SolverException();
	}

	size_t N = this->mRightSideVector.rows();
	auto hMat = this->mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)];

	mGpuRhsVec = cuda::Vector<double>(N);
	mGpuLhsVec = cuda::Vector<double>(N);
	mGpuIntermediateVec = cuda::Vector<double>(N);

	cusparseMatDescr_t descr_M = 0;
    csrilu02Info_t info_M  = 0;
    int structural_zero;
    int numerical_zero;

	size_t nnz = hMat[0].nonZeros();

    // step 1: create a descriptor which contains
    // - matrix M is base-1
    // - matrix L is base-1
    // - matrix L is lower triangular
    // - matrix L has unit diagonal
    // - matrix U is base-1
    // - matrix U is upper triangular
    // - matrix U has non-unit diagonal
    checkCusparseStatus(cusparseCreateMatDescr(&descr_M));
	checkCusparseStatus(cusparseSetMatIndexBase(descr_M, CUSPARSE_INDEX_BASE_ZERO));
    checkCusparseStatus(cusparseSetMatType(descr_M, CUSPARSE_MATRIX_TYPE_GENERAL));

    checkCusparseStatus(cusparseCreateMatDescr(&descr_L));
    checkCusparseStatus(cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ZERO));
    checkCusparseStatus(cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL));
    checkCusparseStatus(cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER));
    checkCusparseStatus(cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_UNIT));

    checkCusparseStatus(cusparseCreateMatDescr(&descr_U));
    checkCusparseStatus(cusparseSetMatIndexBase(descr_U, CUSPARSE_INDEX_BASE_ZERO));
    checkCusparseStatus(cusparseSetMatType(descr_U, CUSPARSE_MATRIX_TYPE_GENERAL));
    checkCusparseStatus(cusparseSetMatFillMode(descr_U, CUSPARSE_FILL_MODE_UPPER));
    checkCusparseStatus(cusparseSetMatDiagType(descr_U, CUSPARSE_DIAG_TYPE_NON_UNIT));

    // step 2: create a empty info structure
    // we need one info for csrilu02 and two info's for csrsv2
    checkCusparseStatus(cusparseCreateCsrilu02Info(&info_M));
    checkCusparseStatus(cusparseCreateCsrsv2Info(&info_L));
    checkCusparseStatus(cusparseCreateCsrsv2Info(&info_U));

	// step 2a: permutate Matrix M' = P*M
	int p_nnz = 0;
	int p[N];

	cso_status = cusolverSpDcsrzfdHost(
		mCusolverhandle, N,nnz, descr_M,
		hMat[0].valuePtr(),
		hMat[0].outerIndexPtr(),
		hMat[0].innerIndexPtr(),
		p, &p_nnz);

	if (cso_status != CUSOLVER_STATUS_SUCCESS) {
		mSLog->error("cusolverSpDcsrzfdHost returend an error");
	}
	// create Eigen::PermutationMatrix from the p
	mTransp = std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic> >(
			new Eigen::PermutationMatrix<Eigen::Dynamic>(
			Eigen::Map< Eigen::Matrix<int, Eigen::Dynamic, 1> >(p, N, 1)));

	// apply permutation
	hMat[0] = *mTransp * hMat[0];

	// copy P' to GPU
    mSysMat = std::unique_ptr<cuda::CudaMatrix<double, int>>(new cuda::CudaMatrix<double, int>(hMat[0], N));

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

	csp_status = cusparseDcsrsv2_bufferSize(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE,
											N, nnz, descr_L, d_csrVal, d_csrRowPtr,
											d_csrColInd, info_L, &pBufferSize_L);
	checkCusparseStatus(csp_status, "failed to get cusparse bufferSize:");

    csp_status = cusparseDcsrsv2_bufferSize(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE,
											N, nnz, descr_U, d_csrVal, d_csrRowPtr,
											d_csrColInd, info_U, &pBufferSize_U);
	checkCusparseStatus(csp_status, "failed to get cusparse bufferSize:");

	// Buffer
    pBufferSize = std::max({pBufferSize_M, pBufferSize_L, pBufferSize_U});
	pBuffer = cuda::Vector<char>(pBufferSize);

    // step 4: perform analysis of incomplete Cholesky on M
    //         perform analysis of triangular solve on L
    //         perform analysis of triangular solve on U
    // The lower(upper) triangular part of M has the same sparsity pattern as L(U),
    // we can do analysis of csrilu0 and csrsv2 simultaneously.
    csp_status = cusparseDcsrilu02_analysis(mCusparsehandle, N, nnz, descr_M,
											d_csrVal, d_csrRowPtr, d_csrColInd,
											info_M, CUSPARSE_SOLVE_POLICY_NO_LEVEL,
											pBuffer.data());
	checkCusparseStatus(csp_status, "failed to analyse cusparse problem:");

    csp_status = cusparseXcsrilu02_zeroPivot(mCusparsehandle, info_M, &structural_zero);
    if (csp_status == CUSPARSE_STATUS_ZERO_PIVOT){
        mSLog->error("A({},{}) is missing", structural_zero, structural_zero);
		checkCusparseStatus(csp_status);
		throw SolverException();
    }

	csp_status = cusparseDcsrsv2_analysis(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE,
										  N, nnz, descr_L, d_csrVal, d_csrRowPtr,
										  d_csrColInd, info_L, CUSPARSE_SOLVE_POLICY_NO_LEVEL,
										  pBuffer.data());
	checkCusparseStatus(csp_status, "failed to analyse cusparse problem:");

    csp_status = cusparseDcsrsv2_analysis(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE,
										  N, nnz, descr_U, d_csrVal, d_csrRowPtr,
										  d_csrColInd, info_U, CUSPARSE_SOLVE_POLICY_USE_LEVEL,
										  pBuffer.data());
	checkCusparseStatus(csp_status, "failed to analyse cusparse problem:");

    // step 5: M = L * U

    csp_status = cusparseDcsrilu02(mCusparsehandle, N, nnz, descr_M, d_csrVal,
								   d_csrRowPtr, d_csrColInd, info_M, CUSPARSE_SOLVE_POLICY_NO_LEVEL,
								   pBuffer.data());
	checkCusparseStatus(csp_status, "failed to perform cusparse ILU:");

    csp_status = cusparseXcsrilu02_zeroPivot(mCusparsehandle, info_M, &numerical_zero);
    if (csp_status == CUSPARSE_STATUS_ZERO_PIVOT){
        mSLog->error("U({},{}) is zero\n", numerical_zero, numerical_zero);
		checkCusparseStatus(csp_status);
		throw SolverException();
    }

    //std::cout << *mSysMat.get() << std::endl;

	csp_status = cusparseDestroyCsrilu02Info(info_M);
	checkCusparseStatus(csp_status, "failed to destroy info:");
	csp_status = cusparseDestroyMatDescr(descr_M);
	checkCusparseStatus(csp_status, "failed to destroy MatDescr:");
}

template <typename VarType>
Task::List MnaSolverGpuSparse<VarType>::getTasks() {
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
	l.push_back(std::make_shared<MnaSolverGpuSparse<VarType>::SolveTask>(*this));
    l.push_back(std::make_shared<MnaSolverGpuSparse<VarType>::LogTask>(*this));
	return l;
}

template <typename VarType>
void MnaSolverGpuSparse<VarType>::solve(Real time, Int timeStepCount) {
	cudaError_t status;
	cusparseStatus_t csp_status;
	int size = this->mRightSideVector.rows();
    // Reset source vector
	this->mRightSideVector.setZero();

    // Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (const auto &stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

	if (!this->mIsInInitialization)
		this->updateSwitchStatus();

    //Copy right vector to device
	//Permutate right side: R' = P * R
	this->mRightSideVector = *mTransp * this->mRightSideVector;
    status = cudaMemcpy(mGpuRhsVec.data(), &this->mRightSideVector(0), size * sizeof(Real), cudaMemcpyHostToDevice);
	if (status != cudaSuccess) {
		mSLog->error("Cuda Error: {}", cudaGetErrorString(status));
		throw SolverException();
	}

	const double alpha = 1.;
    // Solve
	// step 6: solve L*z = x
    csp_status = cusparseDcsrsv2_solve(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE,
									   size, mSysMat->non_zero, &alpha, descr_L,
									   mSysMat->val.data(), mSysMat->row.data(),
									   mSysMat->col.data(), info_L, mGpuRhsVec.data(),
									   mGpuIntermediateVec.data(), CUSPARSE_SOLVE_POLICY_NO_LEVEL,
									   pBuffer.data());
	checkCusparseStatus(csp_status, "failed to solve L*z=x:");

    // step 7: solve U*y = z
    csp_status = cusparseDcsrsv2_solve(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE,
									   size, mSysMat->non_zero, &alpha, descr_U,
									   mSysMat->val.data(), mSysMat->row.data(),
									   mSysMat->col.data(), info_U, mGpuIntermediateVec.data(),
									   mGpuLhsVec.data(), CUSPARSE_SOLVE_POLICY_USE_LEVEL,
									   pBuffer.data());
	checkCusparseStatus(csp_status, "failed to solve U*y=z:");

    //Copy Solution back
    status = cudaMemcpy(&this->mLeftSideVector(0), mGpuLhsVec.data(), size * sizeof(Real), cudaMemcpyDeviceToHost);
	if (status != cudaSuccess) {
		mSLog->error("Cuda Error: {}", cudaGetErrorString(status));
		throw SolverException();
	}

	//Apply inverse Permutation L = P' * L'
	//this->mLeftSideVector = mTransp->inverse() * this->mLeftSideVector;

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
		this->mNodes[nodeIdx]->mnaUpdateVoltage(this->mLeftSideVector);


	// Components' states will be updated by the post-step tasks
}

}
template class DPsim::MnaSolverGpuSparse<Real>;
template class DPsim::MnaSolverGpuSparse<Complex>;
#pragma GCC diagnostic pop
