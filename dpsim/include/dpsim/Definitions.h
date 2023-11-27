/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>

// This macro defines the tolerance used to compare double numbers
#define DOUBLE_EPSILON 1E-12 

namespace DPsim {
	// #### Types ####
	using Real = CPS::Real;
	using Complex = CPS::Complex;
	using String = CPS::String;
	using Bool = CPS::Bool;
	using Int = CPS::Int;
	using UInt = CPS::UInt;
	using Matrix = CPS::Matrix;
	using MatrixComp = CPS::MatrixComp;
	using SparseMatrix = CPS::SparseMatrixRow;
	using SparseMatrixComp = CPS::SparseMatrixCompRow;

	template<typename T>
	using MatrixVar = CPS::MatrixVar<T>;

	class SolverException { };
	class UnsupportedSolverException { };
}
