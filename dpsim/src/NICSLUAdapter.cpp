/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/NICSLUAdapter.h>

using namespace DPsim;

namespace DPsim
{
	NICSLUAdapter::~NICSLUAdapter()
	{
		if(nicslu)
		{
			NicsLU_Destroy(nicslu);
			free(nicslu);
		}
	}

	NICSLUAdapter::NICSLUAdapter()
	{
		nicslu = (SNicsLU*)malloc(sizeof(SNicsLU));
		NicsLU_Initialize(nicslu);
		 /* here: NICSLU parameters are set */
		/* scaling modes, etc. */
		nicslu->cfgi[0] = 0;
		nicslu->cfgi[1] = 1;
		nicslu->cfgi[10] = 0;

		/* setting pivoting tolerance for refatorization */
		nicslu->cfgf[31] = 1e-8;
		char *pivot_tolerance_env = getenv("NICSLU_PIVOT_TOLERANCE");
		if (pivot_tolerance_env != NULL) {
		double pivot_tolerance = atof(pivot_tolerance_env);
		if (pivot_tolerance > 0)
			nicslu->cfgf[31] = pivot_tolerance;
		}
		char *nicslu_do_mc64 = getenv("NICSLU_MC64");
		if (nicslu_do_mc64 != NULL) {
		nicslu->cfgi[1] = atoi(nicslu_do_mc64);
		}
		char *nicslu_scale = getenv("NICSLU_SCALE");
		if (nicslu_scale != NULL) {
		nicslu->cfgi[2] = atoi(nicslu_scale);
		}
	}

	void NICSLUAdapter::preprocessing(SparseMatrix &systemMatrix, std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries)
	{
		const uint__t n = Eigen::internal::convert_index<uint__t>(systemMatrix.rows());
		auto Ap = Eigen::internal::convert_index<uint__t *>(systemMatrix.outerIndexPtr());
		auto Ai = Eigen::internal::convert_index<uint__t *>(systemMatrix.innerIndexPtr());
		real__t* Ax = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());
	    uint__t nnz = Eigen::internal::convert_index<uint__t>(systemMatrix.nonZeros());


		this->changedEntries = listVariableSystemMatrixEntries;
    	Int varying_entries = Eigen::internal::convert_index<Int>(changedEntries.size());
    	std::vector<uint__t> varying_columns;
    	std::vector<uint__t> varying_rows;

		for (auto &changedEntry : changedEntries)
		{
			varying_rows.push_back(changedEntry.first);
			varying_columns.push_back(changedEntry.second);
		}

		NicsLU_CreateMatrix(nicslu, n, nnz, Ax, Ai, Ap);
		NicsLU_Analyze(nicslu, &varying_rows[0], &varying_columns[0], varying_entries);
	}

	void NICSLUAdapter::factorize(SparseMatrix &systemMatrix)
	{
		real__t* Az = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());
		NicsLU_ResetMatrixValues(nicslu, Az);
		NicsLU_Factorize(nicslu);
		if(nicslu->cfgi[10] == 0 || nicslu->cfgi[10] == 1)
		{
			/* factorization path required here */
			NicsLU_compute_path(nicslu);
		}
		else
		{
			/* no factorization path required, doing refactorization restart */
			/* this function essentially tells NICSLU where to restart the refactorization */
			NicsLU_RefactorizationRestart(nicslu);
		}
	}

	void NICSLUAdapter::refactorize(SparseMatrix &systemMatrix)
	{
		if (systemMatrix.nonZeros() != nicslu->nnz) {
			preprocessing(systemMatrix, this->changedEntries);
        	factorize(systemMatrix);
		} else {
			// get new matrix values
			real__t* Ax = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());

			// Refactorize with new values
			int numOk = NicsLU_ReFactorize(nicslu, Ax);

			// check whether a pivot became too large or too small
			if (numOk == NICSLU_NUMERIC_OVERFLOW) {
				// if so, reset matrix values and re-do computation
				// only needs to Reset Matrix Values, if analyze_pattern is not called
				numOk = NicsLU_ResetMatrixValues(nicslu, Ax);
				numOk = NicsLU_Factorize(nicslu);
			}
		}
	}

	void NICSLUAdapter::partialRefactorize(SparseMatrix &systemMatrix, std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries)
	{
		if (systemMatrix.nonZeros() != nnz)
		{
			preprocessing(systemMatrix, listVariableSystemMatrixEntries);
			factorize(systemMatrix);
		}
		else
		{
			if(nicslu->cfgi[10] == 0 || nicslu->cfgi[10] == 1)
				{
					/* factorization path mode */

					/* get new matrix values */
					real__t* Ax = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());

					/* Refactorize partially with new values */
					int numOk = NicsLU_PartialReFactorize(nicslu, Ax);

					/* check whether a pivot became too large or too small */
					if (numOk == NICSLU_NUMERIC_OVERFLOW)
					{
						/* if so, reset matrix values and re-do factorization
						* only needs to Reset Matrix Values, if analyze_pattern is not called
						*/
						numOk = NicsLU_ResetMatrixValues(nicslu, Ax);
						numOk = NicsLU_Factorize(nicslu);
					}
				}
				else
				{
					// restarting partial refactorisation
					// either BRA or AMD ordering ("canadian method")

					// get new matrix values
					real__t* Ax = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());

					// Refactorize with new values
					int numOk = NicsLU_FPartialReFactorize(nicslu, Ax);

					// check whether a pivot became too large or too small
					if (numOk == NICSLU_NUMERIC_OVERFLOW)
					{
						// if so, reset matrix values and re-do computation
						// only needs to Reset Matrix Values, if analyze_pattern is not called
						numOk = NicsLU_ResetMatrixValues(nicslu, Ax);
						numOk = NicsLU_Factorize(nicslu);
					}
				}
		}
	}

	Matrix NICSLUAdapter::solve(Matrix &rightSideVector)
	{
		/* TODO: ensure matrix has been factorized properly before calling this function.
		* assertions might hurt performance, thus omitted here */

		Matrix x = rightSideVector;

		NicsLU_Solve(nicslu, x.const_cast_derived().data());

		return x;
	}
}
