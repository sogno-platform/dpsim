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
		// here: NICSLU parameters are set
		nicslu->cfgi[0] = NICSLU_CSR_STORAGE;
		// to use mc64 or not
		nicslu->cfgi[1] = 1;
		// (Partial) Ordering
		nicslu->cfgi[10] = NICSLU_AMD_ORDERING;
		// setting pivoting tolerance for refatorization
		nicslu->cfgf[31] = 1e-8;
	}

	NICSLUAdapter::NICSLUAdapter(CPS::Logger::Log log) : NICSLUAdapter()
	{
		this->mSLog = log;
	}

	void NICSLUAdapter::preprocessing(SparseMatrix &systemMatrix, std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries)
	{
		const uint__t n = Eigen::internal::convert_index<uint__t>(systemMatrix.rows());
		auto Ap = Eigen::internal::convert_index<uint__t *>(systemMatrix.outerIndexPtr());
		auto Ai = Eigen::internal::convert_index<uint__t *>(systemMatrix.innerIndexPtr());
		real__t* Ax = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());
	    uint__t _nnz = Eigen::internal::convert_index<uint__t>(systemMatrix.nonZeros());


		this->changedEntries = listVariableSystemMatrixEntries;
    	Int varying_entries = Eigen::internal::convert_index<Int>(changedEntries.size());
    	std::vector<uint__t> varying_columns;
    	std::vector<uint__t> varying_rows;

		for (auto &changedEntry : changedEntries)
		{
			varying_rows.push_back(changedEntry.first);
			varying_columns.push_back(changedEntry.second);
		}

		NicsLU_CreateMatrix(nicslu, n, _nnz, Ax, Ai, Ap);
		NicsLU_Analyze(nicslu, &varying_rows[0], &varying_columns[0], varying_entries);

		/* store non-zero value of current preprocessed matrix. only used until
			* to-do in refactorize-function is resolved. Can be removed then. */
		nnz = Eigen::internal::convert_index<Int>(systemMatrix.nonZeros());
	}

	void NICSLUAdapter::factorize(SparseMatrix &systemMatrix)
	{
		real__t* Az = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());
		NicsLU_ResetMatrixValues(nicslu, Az);
		NicsLU_Factorize(nicslu);

		// FIXME: The mode of partial refactorization shall be determined by the user using the configuration class and not the ordering method
		// also fix down below
		if(mPartialRefactorizationMethod == PARTIAL_REFACTORIZATION_METHOD::FACTORIZATION_PATH)
		{
			/* factorization path required here */
			NicsLU_compute_path(nicslu);
		}
		else if(mPartialRefactorizationMethod == PARTIAL_REFACTORIZATION_METHOD::REFACTORIZATION_RESTART)
		{
			/* no factorization path required, doing refactorization restart */
			/* this function essentially tells NICSLU where to restart the refactorization */
			NicsLU_RefactorizationRestart(nicslu);
		}
	}

	void NICSLUAdapter::refactorize(SparseMatrix &systemMatrix)
	{
		if (Eigen::internal::convert_index<Int>(systemMatrix.nonZeros()) != nnz)
		{
			preprocessing(systemMatrix, this->changedEntries);
        	factorize(systemMatrix);
		}
		else
		{
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
		if (Eigen::internal::convert_index<Int>(systemMatrix.nonZeros()) != nnz)
		{
			preprocessing(systemMatrix, listVariableSystemMatrixEntries);
			factorize(systemMatrix);
		}
		else
		{
			/* get new matrix values */
			real__t* Ax = Eigen::internal::convert_index<real__t *>(systemMatrix.valuePtr());

			int numOk = NICS_OK;

			if(mPartialRefactorizationMethod == PARTIAL_REFACTORIZATION_METHOD::FACTORIZATION_PATH)
			{
				// factorization path mode

				// Refactorize partially with new values
				numOk = NicsLU_Partial_Factorization_Path(nicslu, Ax);
			}
			else if(mPartialRefactorizationMethod == PARTIAL_REFACTORIZATION_METHOD::REFACTORIZATION_RESTART)
			{
				// restarting partial refactorisation

				// Refactorize with new values
				numOk = NicsLU_Partial_Refactorization_Restart(nicslu, Ax);
			}
			else
			{
				// no partial refactorization requested
				// NOTE: now the check whether the nnz(A) match is executed twice
				// this will be removed when the issue of matrix nonzeros is fixed.
				refactorize(systemMatrix);
			}

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

	Matrix NICSLUAdapter::solve(Matrix &rightSideVector)
	{
		/* TODO: ensure matrix has been factorized properly before calling this function.
		* assertions might hurt performance, thus omitted here */

		Matrix x = rightSideVector;

		NicsLU_Solve(nicslu, x.const_cast_derived().data());

		return x;
	}

	void NICSLUAdapter::applyConfiguration()
	{
		// decide whether use MC64, a weighted maximum-matching/scaling algorithm
		switch(mConfiguration.getMC64())
		{
			case USE_MC64::DO_MC64:
				nicslu->cfgi[1] = 1;
				break;
			case USE_MC64::NO_MC64:
				nicslu->cfgi[1] = 0;
				break;
			default:
				nicslu->cfgi[1] = 1; // use MC64 by default
		}

		SPDLOG_LOGGER_INFO(mSLog, "Matrix is permuted and scaled " + mConfiguration.getMC64String());

		// decide whether to use additional scaling during factorization. default is 0
		// according to NICSLU doc: different scaling methods may have effect in frequency domain simulation
		// but no effect in time-domain transient simulation
		switch(mConfiguration.getScalingMethod())
		{
			case SCALING_METHOD::MAX_SCALING:
				nicslu->cfgi[2] = 1;
				break;
			case SCALING_METHOD::SUM_SCALING:
				nicslu->cfgi[2] = 2;
				break;
			case SCALING_METHOD::NO_SCALING:
			default:
				nicslu->cfgi[2] = 0;
		}

		if(nicslu->cfgi[2] != 0)
		{
			SPDLOG_LOGGER_INFO(mSLog, "Matrix is additionally scaled using " + mConfiguration.getScalingMethodString());
		}
		else
		{
			SPDLOG_LOGGER_INFO(mSLog, "Matrix is not additionally scaled");
		}

		switch(mConfiguration.getFillInReductionMethod())
		{
		case FILL_IN_REDUCTION_METHOD::AMD:
			nicslu->cfgi[10] = NICSLU_AMD_ORDERING;
			break;
		case FILL_IN_REDUCTION_METHOD::AMD_NV:
			nicslu->cfgi[10] = NICSLU_AMD_NV_ORDERING;
			break;
		case FILL_IN_REDUCTION_METHOD::AMD_RA:
			nicslu->cfgi[10] = NICSLU_AMD_RA_ORDERING;
			break;
		default:
			nicslu->cfgi[10] = NICSLU_AMD_ORDERING;
		}

		SPDLOG_LOGGER_INFO(mSLog, "Matrix is fill reduced with " + mConfiguration.getFillInReductionMethodString());

		// NOTE: in case more partial refactorization methods are defined/developed, that are not implemented in NICSLU, this assigment would be invalid
		mPartialRefactorizationMethod = mConfiguration.getPartialRefactorizationMethod();

		SPDLOG_LOGGER_INFO(mSLog, "Matrix is refactored " + mConfiguration.getPartialRefactorizationMethodString());
	}
}
