#pragma once

#include <dpsim/MNASolver.h>
#include <dpsim/Cuda/CudaUtility.h>

namespace DPsim {

	template <typename VarType>
    class MnaSolverGpuSparse : public MnaSolver<VarType>{
	protected:

		// #### Attributes required for GPU ####
		/// Solver-Handle
		cusparseHandle_t mCusparsehandle;

		/// Systemmatrix on Device
		std::unique_ptr<cuda::CudaMatrix<double, int>> mSysMat;

		/// RHS-Vector
		cuda::Vector<double> mGpuRhsVec;
		/// LHS-Vector
		cuda::Vector<double> mGpuLhsVec;
		/// Intermediate Vector
		cuda::Vector<double> mGpuIntermediateVec;

		/// Initialize cuSparse-library
        void initialize();
		/// ILU factorization
		void iluPreconditioner();
		///
		void solve(Real time, Int timeStepCount);

	private:
		///Required shared Variables
		cuda::Vector<char> pBuffer;
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
