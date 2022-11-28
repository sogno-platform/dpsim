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

#include <dpsim/Cuda/CudaUtility.h>
#include <cusolverSp.h>

namespace DPsim
{
	class GpuSparseAdapter : public DirectLinearSolver
    {
		private:
		///Required shared Variables
		cuda::Vector<char> pBuffer;
		cusparseMatDescr_t descr_L = nullptr;
		cusparseMatDescr_t descr_U = nullptr;
		csrsv2Info_t info_L = nullptr;
		csrsv2Info_t info_U = nullptr;

		void checkCusparseStatus(cusparseStatus_t status, std::string additionalInfo="cuSparse Error:");
		
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

		void iluPreconditioner();

        public:
		
		/// Destructor
		virtual ~GpuSparseAdapter();

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
