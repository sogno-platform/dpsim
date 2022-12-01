/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

extern "C"
{
    #include <klu.h>
}

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <bitset>
#include <memory>
#include <fstream>

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/DirectLinearSolver.h>

namespace DPsim
{

    class KLUAdapter : public DirectLinearSolver
    {
        /// Vector of variable entries in system matrix
        std::vector<std::pair<UInt, UInt>> changedEntries;

		/// KLU-specific structs
        klu_common m_common;
		klu_numeric* m_numeric;
		klu_symbolic* m_symbolic;
		
		/// Flags to indicate mode of operation
		/// Define which ordering to choose in preprocessing
		int m_ordering;

		/// Use BTF or not
		int m_btf;

		/// Use scaling or not
		int m_scaling;

		/// Dump the system matrix into a file or not
		int m_dump;

		/// If so, indicate second matrix to dump (the m_limit-th matrix will be dumped and the first)
		int m_limit;

		/// Flag to indicate if factorization succeeded
		bool factorization_is_okay;

		/// Flag to indicate if preprocessing succeeded
		bool preprocessing_is_okay;

		/// Temporary value to store the number of nonzeros
		int nnz;

        public:
		/// Destructor
		virtual ~KLUAdapter();

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

		protected:

		/// Function to print matrix 
		void printMTX(const SparseMatrix& matrix, int counter);
    };
}
