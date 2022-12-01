/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <bitset>
#include <memory>

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/DirectLinearSolver.h>

#include <cuda_runtime.h>
#include <cusolverDn.h>

#define CUDA_ERROR_HANDLER(func) {cudaError_t error; if((error = func) != cudaSuccess) std::cerr << cudaGetErrorString(error) << std::endl; }

namespace DPsim
{
	class GpuDenseAdapter : public DirectLinearSolver
    {
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

		void allocateDeviceMemory();
		
		void copySystemMatrixToDevice(Matrix mVariableSystemMatrix);
		
		void LUfactorization();

        public:
		
		/// Destructor
		virtual ~GpuDenseAdapter();

		/// initialization function for linear solver
		virtual void initialize() override;

		/// preprocessing function pre-ordering and scaling the matrix
		virtual void preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) override;

		/// factorization function with partial pivoting
		virtual void factorize(SparseMatrix& mVariableSystemMatrix) override;

		/// refactorization without partial pivoting
		virtual void refactorize(SparseMatrix& mVariableSystemMatrix) override;

		/// partial refactorization withouth partial pivoting
		virtual void partialRefactorize(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) override;

		/// solution function for a right hand side
		virtual Matrix solve(Matrix& mRightSideVector) override;
    };
}