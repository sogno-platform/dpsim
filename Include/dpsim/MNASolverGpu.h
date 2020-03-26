#pragma once

#include <dpsim/MNASolver.h>

#include <cuda_runtime.h>
#include <cusolverDn.h>

/**
 * 
 * TODO:
 *    -initialize();
 *    -void setSystem(CPS::SystemTopology system);
 *    -CPS::Task::List getTasks();
 *    -class SolveTask : public CPS::Task
 * 	  -class LogTask : public CPS::Task
 * 
 */

namespace DPsim {
	template <typename VarType>
    class MnaSolverGpu : public MnaSolver<VarType>{
	protected:
		// #### Attributes required for GPU ####
		///Sovler-Handle
		cusolverDnHandle_t mCusolverHandle;
		///Stream
		cudaStream_t mStream;

		/// Variables for solving one Equation-system
		struct GpuData {
			/// Device copy of System-Matrix
			double *matrix;
			/// Device copy of Vector
			double *rightVector;

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
		

	public:
		MnaSolverGpu();

		virtual ~MnaSolverGpu();


		class SolveTask : public MnaSolver::SolveTask {
			SolveTask(MnaSolver<VarType>& solver, Bool steadyStateInit) :
			MnaSolver::SolveTask(MnaSolver<VarType>& solver, Bool steadyStateInit) {
			}

			void execute(Real time, Int timeStepCount);
		};
    };
}