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
		/* This class follows the rule of three */

		protected:
		/// Copy Constructor
		DirectLinearSolver(const DirectLinearSolver&) = default;

		/// Copy Assignment Operator
		DirectLinearSolver& operator=(const DirectLinearSolver&) = default;

		public:
		/// Constructor
		DirectLinearSolver() = default;

		/// Destructor
		virtual ~DirectLinearSolver() = default;

		/// initialization function for linear solver
		virtual void initialize()
		{
			/* no default initialization
			   FIXME: removable. initialization can be done by constructor
			*/
		};

		/// preprocessing function pre-ordering and scaling the matrix
		virtual void preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) = 0;

		/// factorization function with partial pivoting
		virtual void factorize(SparseMatrix& mVariableSystemMatrix) = 0;

		/// refactorization without partial pivoting
		virtual void refactorize(SparseMatrix& mVariableSystemMatrix) = 0;

		/// partial refactorization withouth partial pivoting
		virtual void partialRefactorize(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries) = 0;

		/// solution function for a right hand side
		virtual Matrix solve(Matrix& mRightSideVector) = 0;
	};
}
