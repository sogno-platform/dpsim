/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/MNASolverEigenDense.h>

#include <cuda_runtime.h>
#include <cusolverDn.h>

#define CUDA_ERROR_HANDLER(func) {cudaError_t error; if((error = func) != cudaSuccess) std::cerr << cudaGetErrorString(error) << std::endl; }

namespace DPsim {
	template <typename VarType>
    class MnaSolverGpuDense : public MnaSolverEigenDense<VarType>{
	protected:

		// #### Attributes required for GPU ####
		/// Solver-Handle
		cusolverDnHandle_t mCusolverHandle;
		/// Stream
		cudaStream_t mStream;

		/// Variables for solving one Equation-system (All pointer are device-pointer)
		struct GpuData {
			/// Device copy of System-Matrix
			double *matrix;
			/// Size of one dimension
			UInt size;
			/// Device copy of Vector
			double *vector;

			/// Device-Workspace for getrf
			double *workSpace;
			/// Pivoting-Sequence
			int *pivSeq;
			/// Errorinfo
			int *errInfo;
		} mDeviceCopy;

		/// Initialize cuSolver-library
        void initialize();
        /// Allocate Space for Vectors & Matrices on GPU
        void allocateDeviceMemory();
		/// Copy Systemmatrix to Device
		void copySystemMatrixToDevice();
		/// LU factorization
		void LUfactorization();
		///
		void solve(Real time, Int timeStepCount);

	public:
		MnaSolverGpuDense(String name,
			CPS::Domain domain = CPS::Domain::DP,
			CPS::Logger::Level logLevel = CPS::Logger::Level::info);

		virtual ~MnaSolverGpuDense();

		CPS::Task::List getTasks();

		class SolveTask : public CPS::Task {
		public:
			SolveTask(MnaSolverGpuDense<VarType>& solver) :
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
			MnaSolverGpuDense<VarType>& mSolver;
		};

		class LogTask : public CPS::Task {
		public:
			LogTask(MnaSolverGpuDense<VarType>& solver) :
				Task(solver.mName + ".Log"), mSolver(solver) {
				mAttributeDependencies.push_back(solver.attribute("left_vector"));
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) { mSolver.log(time, timeStepCount); }

		private:
			MnaSolverGpuDense<VarType>& mSolver;
		};
    };
}
