#pragma once

#include <dpsim/MNASolverEigenSparse.h>

#include <magma_v2.h>
#include <magmasparse.h>


namespace DPsim {

	template <typename VarType>
    class MnaSolverGpuMagma : public MnaSolverEigenSparse<VarType>{
	protected:

		std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic>> mTransp;
		// #### Attributes required for GPU ####
		/// Solver-Handle
		magma_dopts mMagmaOpts;
		magma_queue_t mMagmaQueue;

		/// Systemmatrix
		magma_d_matrix mHostSysMat;
		magma_d_matrix mDevSysMat;

		/// RHS-Vector
		magma_d_matrix mHostRhsVec;
		magma_d_matrix mDevRhsVec;
		/// LHS-Vector
		magma_d_matrix mHostLhsVec;
		magma_d_matrix mDevLhsVec;

		using Solver::mSLog;

		/// Initialize cuSparse-library
        void initialize() override;
		/// ILU factorization
		void iluPreconditioner();
		///
		void solve(Real time, Int timeStepCount) override;

	private:

	public:
		MnaSolverGpuMagma(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		virtual ~MnaSolverGpuMagma();

		CPS::Task::List getTasks() override;

		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolverGpuMagma<VarType>& solver) :
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
			MnaSolverGpuMagma<VarType>& mSolver;
		};

		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolverGpuMagma<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.attribute("left_vector"));
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolverGpuMagma<VarType>& mSolver;
		};
    };

}
