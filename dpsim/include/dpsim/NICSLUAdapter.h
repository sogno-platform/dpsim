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
    #include <nicslu.h>
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
    class NICSLUAdapter : public DirectLinearSolver
    {
        /// Vector of variable entries in system matrix
        std::vector<std::pair<UInt, UInt>> changedEntries;

		/// NICSLU-specific structs
		SNicsLU* nicslu = nullptr;

		/// Flags to indicate mode of operation
		/// Define which ordering to choose in preprocessing
		PARTIAL_REFACTORIZATION_METHOD mPartialRefactorizationMethod = PARTIAL_REFACTORIZATION_METHOD::FACTORIZATION_PATH;

		/// Temporary value to store the number of nonzeros
		Int nnz;

    public:
		/// Destructor
		~NICSLUAdapter() override;

		/// Constructor
		NICSLUAdapter();

		/// Constructor with logging
		NICSLUAdapter(CPS::Logger::Log log);

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

		/// Apply configuration
		void applyConfiguration() override;
    };
}
