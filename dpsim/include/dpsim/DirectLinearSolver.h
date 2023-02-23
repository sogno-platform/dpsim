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

#include <dpsim/Config.h>
#include <dpsim/Definitions.h>

namespace DPsim
{
	/// Abstract linear solver class for MNA simulation

	class DirectLinearSolver
	{

		public:
		/// Constructor
		DirectLinearSolver() = default;

		/// Destructor
		virtual ~DirectLinearSolver() = default;

		/// Copy Constructor
		DirectLinearSolver(const DirectLinearSolver&) = default;

		/// Copy Assignment Operator
		DirectLinearSolver& operator=(const DirectLinearSolver&) = default;

		/// Move Constructor
		DirectLinearSolver(DirectLinearSolver&&) = default;

		/// Move Assignment Operator
		DirectLinearSolver& operator=(DirectLinearSolver&&) = default;

		/// preprocessing function pre-ordering and scaling the matrix
		virtual void preprocessing(SparseMatrix& systemMatrix, std::vector<std::pair<UInt, UInt>>& listVariableSystemMatrixEntries) = 0;

		/// factorization function with partial pivoting
		virtual void factorize(SparseMatrix& systemMatrix) = 0;

		/// refactorization without partial pivoting
		virtual void refactorize(SparseMatrix& systemMatrix) = 0;

		/// partial refactorization withouth partial pivoting
		virtual void partialRefactorize(SparseMatrix& systemMatrix, std::vector<std::pair<UInt, UInt>>& listVariableSystemMatrixEntries) = 0;

		/// solution function for a right hand side
		virtual Matrix solve(Matrix& rightSideVector) = 0;
	};
}
