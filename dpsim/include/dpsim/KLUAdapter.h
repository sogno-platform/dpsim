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
		klu_numeric* m_numeric = nullptr;
		klu_symbolic* m_symbolic = nullptr;

		/// Flags to indicate mode of operation
		/// Define which ordering to choose in preprocessing
		int m_ordering = KLU_AMD_FP;

		/// Use BTF or not
		int m_btf = 1;

		/// Use scaling or not
		int m_scaling = 1;

		/// Flag to indicate if factorization succeeded
		bool factorization_is_okay = false;

		/// Flag to indicate if preprocessing succeeded
		bool preprocessing_is_okay = false;

		/// Temporary value to store the number of nonzeros
		Int nnz;

    public:
		/// Destructor
		~KLUAdapter() override;

		/// Constructor
		KLUAdapter();

		/// preprocessing function pre-ordering and scaling the matrix
		void preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) override;

		/// factorization function with partial pivoting
		void factorize(SparseMatrix& mVariableSystemMatrix) override;

		/// refactorization without partial pivoting
		void refactorize(SparseMatrix& mVariableSystemMatrix) override;

		/// partial refactorization withouth partial pivoting
		void partialRefactorize(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) override;

		/// solution function for a right hand side
		Matrix solve(Matrix& mRightSideVector) override;

		protected:

		/// Function to print matrix
		void printMTX(SparseMatrix& matrix, int counter) const;
    };
}
