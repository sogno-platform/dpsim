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
	enum SCALING_METHOD {
		NO_SCALING = 0,
		SUM_SCALING = 1,
		MAX_SCALING = 2
	};

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
		int m_partial_method = KLU_AMD_FP;

		/// Use BTF or not
		bool m_btf = true;

		/// Define scaling method
		/// Options are: 1: sum-scaling, 2: max scaling, <=0: no scaling
		int m_scaling = SCALING_METHOD::MAX_SCALING;

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
		void preprocessing(SparseMatrix& systemMatrix, std::vector<std::pair<UInt, UInt>>& listVariableSystemMatrixEntries) override;

		/// factorization function with partial pivoting
		void factorize(SparseMatrix& systemMatrix) override;

		/// refactorization without partial pivoting
		void refactorize(SparseMatrix& systemMatrix) override;

		/// partial refactorization withouth partial pivoting
		void partialRefactorize(SparseMatrix& systemMatrix, std::vector<std::pair<UInt, UInt>>& listVariableSystemMatrixEntries) override;

		/// solution function for a right hand side
		Matrix solve(Matrix& rightSideVector) override;

		protected:

		/// Function to print matrix in MatrixMarket's coo format
		void printMatrixMarket(SparseMatrix& systemMatrix, int counter) const;
    };
}
