/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/MNASolverEigenSparse.h>
#include <dpsim/Cuda/CudaUtility.h>
#include <cusolverSp.h>

namespace DPsim {

	template <typename VarType>
    class MnaSolverGpuSparse : public MnaSolverEigenSparse<VarType>{
	protected:

		// #### Attributes required for GPU ####
		/// Solver-Handle
		cusparseHandle_t mCusparsehandle;
		cusolverSpHandle_t mCusolverhandle;

		/// Systemmatrix on Device
		std::unique_ptr<cuda::CudaMatrix<double, int>> mSysMat;
		std::unique_ptr<Eigen::PermutationMatrix<Eigen::Dynamic>> mTransp;

		/// RHS-Vector
		cuda::Vector<double> mGpuRhsVec;
		/// LHS-Vector
		cuda::Vector<double> mGpuLhsVec;
		/// Intermediate Vector
		cuda::Vector<double> mGpuIntermediateVec;

		using Solver::mSLog;

		/// Initialize cuSparse-library
        void initialize() override;
		/// ILU factorization
		void iluPreconditioner();
		///
		void solve(Real time, Int timeStepCount) override;

	private:
		///Required shared Variables
		cuda::Vector<char> pBuffer;
		cusparseMatDescr_t descr_L = nullptr;
		cusparseMatDescr_t descr_U = nullptr;
		csrsv2Info_t info_L = nullptr;
		csrsv2Info_t info_U = nullptr;

		void checkCusparseStatus(cusparseStatus_t status, std::string additionalInfo="cuSparse Error:");

	public:
		MnaSolverGpuSparse(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		virtual ~MnaSolverGpuSparse();

		CPS::Task::List getTasks() override;

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

}
