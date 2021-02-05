#include <dpsim/MNASolverGpuSparse.h>
#include <dpsim/SequentialScheduler.h>
#include <Eigen/Eigen>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverGpuSparse<VarType>::MnaSolverGpuSparse(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolver<VarType>(name, domain, logLevel),
    mCusparsehandle(nullptr), mSysMat(nullptr),
	mTransp(nullptr),
	mGpuRhsVec(0), mGpuLhsVec(0), mGpuIntermediateVec(0),
	pBuffer(0){

	//TODO Error-Handling
    cusparseCreate(&mCusparsehandle);
	//TODO: Error-Handling
	cusolverSpCreate(&mCusolverhandle);
}

template <typename VarType>
MnaSolverGpuSparse<VarType>::~MnaSolverGpuSparse() {
    //TODO Deconstructor
}

template <typename VarType>
void MnaSolverGpuSparse<VarType>::initialize() {
    MnaSolver<VarType>::initialize();

    int dim = this->mRightSideVector.rows();
	auto hMat = this->mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)];

	mGpuRhsVec = cuda::Vector<double>(dim);
	mGpuLhsVec = cuda::Vector<double>(dim);
	mGpuIntermediateVec = cuda::Vector<double>(dim);

	cusparseMatDescr_t descr_M = 0;
    csrilu02Info_t info_M  = 0;
    int structural_zero;
    int numerical_zero;

	int N = this->mRightSideVector.rows();
	int nnz = hMat.nonZeros();

    // step 1: create a descriptor which contains
    // - matrix M is base-1
    // - matrix L is base-1
    // - matrix L is lower triangular
    // - matrix L has unit diagonal
    // - matrix U is base-1
    // - matrix U is upper triangular
    // - matrix U has non-unit diagonal
    cusparseCreateMatDescr(&descr_M);
    cusparseSetMatIndexBase(descr_M, CUSPARSE_INDEX_BASE_ZERO);
    cusparseSetMatType(descr_M, CUSPARSE_MATRIX_TYPE_GENERAL);

    cusparseCreateMatDescr(&descr_L);
    cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ZERO);
    cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER);
    cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_UNIT);

    cusparseCreateMatDescr(&descr_U);
    cusparseSetMatIndexBase(descr_U, CUSPARSE_INDEX_BASE_ZERO);
    cusparseSetMatType(descr_U, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatFillMode(descr_U, CUSPARSE_FILL_MODE_UPPER);
    cusparseSetMatDiagType(descr_U, CUSPARSE_DIAG_TYPE_NON_UNIT);

    // step 2: create a empty info structure
    // we need one info for csrilu02 and two info's for csrsv2
    cusparseCreateCsrilu02Info(&info_M);
    cusparseCreateCsrsv2Info(&info_L);
    cusparseCreateCsrsv2Info(&info_U);

	// step 2a: permutate Matrix M' = P*M
	//int p[N];
	int p_nnz = 0;
	int p[N];

	cusolverStatus_t ret = cusolverSpDcsrzfdHost(
		mCusolverhandle, N,nnz, descr_M,
		hMat.valuePtr(),
		hMat.outerIndexPtr(),
		hMat.innerIndexPtr(),
		p, &p_nnz);

	if (ret != CUSOLVER_STATUS_SUCCESS) {
		this->mSLog->error("cusolverSpDcsrzfdHost returend an error");
	}


	mTransp = std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic> >(
			new Eigen::PermutationMatrix<Eigen::Dynamic>(
			Eigen::Map< Eigen::Matrix<int, Eigen::Dynamic, 1> >(p, N, 1)));

	//std::cout << "Transposition:" << std::endl;
	//std::cout << mTransp->indices() << std::endl;

	hMat = *mTransp * hMat;

    mSysMat = std::unique_ptr<cuda::CudaMatrix<double, int>>(new cuda::CudaMatrix<double, int>(hMat, dim));

	double *d_csrVal = mSysMat->val.data();
	int *d_csrRowPtr = mSysMat->row.data();
	int *d_csrColInd = mSysMat->col.data();


    // step 3: query how much memory used in csrilu02 and csrsv2, and allocate the buffer
	int pBufferSize_M, pBufferSize_L, pBufferSize_U;
	int pBufferSize;
    cusparseDcsrilu02_bufferSize(mCusparsehandle, N, nnz, descr_M, d_csrVal, d_csrRowPtr, d_csrColInd, info_M, &pBufferSize_M);
    cusparseDcsrsv2_bufferSize(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_csrVal, d_csrRowPtr, d_csrColInd, info_L, &pBufferSize_L);
    cusparseDcsrsv2_bufferSize(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_U, d_csrVal, d_csrRowPtr, d_csrColInd, info_U, &pBufferSize_U);

	// Buffer
    pBufferSize = std::max({pBufferSize_M, pBufferSize_L, pBufferSize_U});
	pBuffer = cuda::Vector<char>(pBufferSize);

    // step 4: perform analysis of incomplete Cholesky on M
    //         perform analysis of triangular solve on L
    //         perform analysis of triangular solve on U
    // The lower(upper) triangular part of M has the same sparsity pattern as L(U),
    // we can do analysis of csrilu0 and csrsv2 simultaneously.
    cusparseDcsrilu02_analysis(mCusparsehandle, N, nnz, descr_M, d_csrVal, d_csrRowPtr, d_csrColInd, info_M, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer.data());
    auto status = cusparseXcsrilu02_zeroPivot(mCusparsehandle, info_M, &structural_zero);
    if (CUSPARSE_STATUS_ZERO_PIVOT == status){
        std::cout << "A(" << structural_zero << ',' << structural_zero << ") is missing\n" << std::endl;
    }
    cusparseDcsrsv2_analysis(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_L, d_csrVal, d_csrRowPtr, d_csrColInd, info_L, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer.data());
    cusparseDcsrsv2_analysis(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, N, nnz, descr_U, d_csrVal, d_csrRowPtr, d_csrColInd, info_U, CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer.data());

    // step 5: M = L * U

    cusparseDcsrilu02(mCusparsehandle, N, nnz, descr_M, d_csrVal, d_csrRowPtr, d_csrColInd, info_M, CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer.data());
    status = cusparseXcsrilu02_zeroPivot(mCusparsehandle, info_M, &numerical_zero);
    if (CUSPARSE_STATUS_ZERO_PIVOT == status){
        printf("U(%d,%d) is zero\n", numerical_zero, numerical_zero);
    }

    //std::cout << *mSysMat.get() << std::endl;
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
	int size = this->mRightSideVector.rows();
    // Reset source vector
	this->mRightSideVector.setZero();

    // Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (const auto &stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

    //Copy right vector to device
	//Permutate right side: R' = P * R
	this->mRightSideVector = *mTransp * this->mRightSideVector;
    cudaMemcpy(mGpuRhsVec.data(), &this->mRightSideVector(0), size * sizeof(Real), cudaMemcpyHostToDevice);

	const double alpha = 1.;
    // Solve
	// step 6: solve L*z = x
    cusparseDcsrsv2_solve(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, size, mSysMat->non_zero, &alpha, descr_L,
    mSysMat->val.data(), mSysMat->row.data(), mSysMat->col.data(), info_L,
    mGpuRhsVec.data(), mGpuIntermediateVec.data(), CUSPARSE_SOLVE_POLICY_NO_LEVEL, pBuffer.data());

    // step 7: solve U*y = z
    cusparseDcsrsv2_solve(mCusparsehandle, CUSPARSE_OPERATION_NON_TRANSPOSE, size, mSysMat->non_zero, &alpha, descr_U,
    mSysMat->val.data(), mSysMat->row.data(), mSysMat->col.data(), info_U,
    mGpuIntermediateVec.data(), mGpuLhsVec.data(), CUSPARSE_SOLVE_POLICY_USE_LEVEL, pBuffer.data());

    //Copy Solution back
    cudaMemcpy(&this->mLeftSideVector(0), mGpuLhsVec.data(), size * sizeof(Real), cudaMemcpyDeviceToHost);

	//Apply inverse Permutation L = P' * L'
	this->mLeftSideVector = mTransp->inverse() * this->mLeftSideVector;

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
		this->mNodes[nodeIdx]->mnaUpdateVoltage(this->mLeftSideVector);

	if (!this->mIsInInitialization)
		this->updateSwitchStatus();

	// Components' states will be updated by the post-step tasks
}

}
