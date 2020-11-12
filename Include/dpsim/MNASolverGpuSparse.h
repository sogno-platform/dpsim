#pragma once

#include <dpsim/MNASolver.h>

#include <cuda_runtime.h>
#include <cusparse_v2.h>

//Error Handling
#define CUDA_ENABLE_ERROR_CHECK

#ifdef CUDA_ENABLE_ERROR_CHECK
#define checkCudaErrors(CUDA_CALL) \
   {cudaError_t code = (cudaError_t)CUDA_CALL; \
   if (code != cudaSuccess) { \
      printf("CUDA Error: %s\n", cudaGetErrorString(code)); \
   }}
#else
#define checkCudaErrors(CUDA_CALL) CUDA_CALL
#endif

namespace DPsim {

	namespace cuda {

		template <typename T>
		void copybackAndPrint(const char *msg, T *ptr, int n) {
			std::vector<T> buffer(n);
			cudaMemcpy(buffer.data(), ptr, n * sizeof(T), cudaMemcpyDeviceToHost);
			std::cout << msg;
			for(T d : buffer) {
				std::cout << d << ' ';
			}
			std::cout << std::endl;
		}

		struct cuda_delete {
			void operator()(void *ptr) {
				cudaFree(ptr);
			}
		};

		template<typename T>
		struct cuda_new {
			T *operator()(size_t n) {
				T *ptr;
				checkCudaErrors(cudaMalloc(&ptr, n * sizeof(T)));
				return ptr;
			}
		};

		template <typename T>
		struct unique_ptr : public std::unique_ptr<T, cuda_delete> {
			unique_ptr():
				std::unique_ptr<T, cuda_delete>() {
			}

			unique_ptr(size_t n):
				std::unique_ptr<T, cuda_delete>(cuda_new<T>()(n)) {
			}

			//"Auto-Dereferencing"
			operator T*() {
				return std::unique_ptr<T, cuda_delete>::get();
			}
		};

		// Matrix (CSR)
		struct CudaMatrix {
			CudaMatrix(const Eigen::SparseMatrix<double, Eigen::RowMajor> &mat, int dim):
				non_zero(mat.nonZeros()),
				row(cuda::unique_ptr<int>(dim + 1)),
				col(cuda::unique_ptr<int>(mat.nonZeros())),
				val(cuda::unique_ptr<double>(mat.nonZeros())) {

				// Copy Matrix (Host -> Device)
				cudaMemcpy(row, mat.outerIndexPtr(), (dim + 1) * sizeof(int), cudaMemcpyHostToDevice);
				cudaMemcpy(col, mat.innerIndexPtr(), non_zero * sizeof(int), cudaMemcpyHostToDevice);
				cudaMemcpy(val, mat.valuePtr(), non_zero * sizeof(double), cudaMemcpyHostToDevice);
			}

			//Matrix Data
			const int non_zero;
			cuda::unique_ptr<int> row;
			cuda::unique_ptr<int> col;
			cuda::unique_ptr<double> val;
		};
	}

	template <typename VarType>
    class MnaSolverGpuSparse : public MnaSolver<VarType>{
	protected:

		// #### Attributes required for GPU ####
		/// Solver-Handle
		cusparseHandle_t mCusparsehandle;

		/// Systemmatrix on Device
		std::unique_ptr<cuda::CudaMatrix> mSysMat;

		/// RHS-Vector
		cuda::unique_ptr<double> mGpuRhsVec;
		/// LHS-Vector
		cuda::unique_ptr<double> mGpuLhsVec;
		/// Intermediate Vector
		cuda::unique_ptr<double> mGpuIntermediateVec;

		/// Initialize cuSparse-library
        void initialize();
		/// ILU factorization
		void iluPreconditioner();
		///
		void solve(Real time, Int timeStepCount);

	private:
		///Required shared Variables
		void *pBuffer = nullptr;
		cusparseMatDescr_t descr_L = nullptr;
    	cusparseMatDescr_t descr_U = nullptr;
		csrsv2Info_t info_L = nullptr;
    	csrsv2Info_t info_U = nullptr;

	public:
		MnaSolverGpuSparse(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		virtual ~MnaSolverGpuSparse();

		CPS::Task::List getTasks();

		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolverGpuSparse<VarType>& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {

				for (auto it : solver.mMNAComponents) {
					if (it->template attribute<Matrix>("right_vector")->get().size() != 0)
						mAttributeDependencies.push_back(it->attribute("right_vector"));
				}
				for (auto node : solver.mNodes) {
					mModifiedAttributes.push_back(node->attribute("v"));
				}
				mModifiedAttributes.push_back(solver.attribute("left_vector"));
			}

			void execute(Real time, Int timeStepCount) { mSolver.solve(time, timeStepCount); }

		private:
			MnaSolverGpuSparse<VarType>& mSolver;
		};

		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolverGpuSparse<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.attribute("left_vector"));
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolverGpuSparse<VarType>& mSolver;
		};
    };

template class DPsim::MnaSolverGpuSparse<Real>;
template class DPsim::MnaSolverGpuSparse<Complex>;

}
