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
		/// Device copy of System-Matrix
		double *mGpuSystemMatrix;
		/// Device copy of Right Vector
		double *mGpuRightVector;
		/// Device copy if Left Vector
		double *mGpuLeftVector;

		/// Initialize cuSolver-library
        void initialize();
        /// Allocate Space for Vectors & Matrices on GPU
        void createEmptyVectors();
	    void createEmptySystemMatrix();

	public:
		MnaSolverGpu();

		virtual ~MnaSolverGpu();
    };
}