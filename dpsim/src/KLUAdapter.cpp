/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/KLUAdapter.h>

using namespace DPsim;

namespace DPsim
{
KLUAdapter::~KLUAdapter()
{
    if (m_symbolic)
        klu_free_symbolic(&m_symbolic, &m_common);
    if (m_numeric)
        klu_free_numeric(&m_numeric, &m_common);
}

KLUAdapter::KLUAdapter(const DirectLinearSolverConfiguration& configuration)
{
	klu_defaults(&m_common);

	switch(configuration.getScalingMethod())
	{
		case SCALING_METHOD::NO_SCALING:
				m_common.scale = 0;
				break;
		case SCALING_METHOD::SUM_SCALING:
				m_common.scale = 1;
				break;
		case SCALING_METHOD::MAX_SCALING:
				m_common.scale = 2;
				break;
		default:
				m_common.scale = 1;
	}

	// TODO: implement support for COLAMD (modifiy SuiteSparse)
	switch(configuration.getFillInReductionMethod())
	{
		case FILL_IN_REDUCTION_METHOD::AMD:
			m_preordering = AMD_ORDERING;
			break;
		case FILL_IN_REDUCTION_METHOD::AMD_NV:
			m_preordering = AMD_ORDERING_NV;
			break;
		case FILL_IN_REDUCTION_METHOD::AMD_RA:
			m_preordering = AMD_ORDERING_RA;
			break;
		default:
			m_preordering = AMD_ORDERING;
	}

	// TODO: implement support for partial refactorization method. Use function pointers?

	switch(configuration.getBTF())
	{
		case USE_BTF::DO_BTF:
				m_common.btf = 1;
				break;
		case USE_BTF::NO_BTF:
				m_common.btf = 0;
				break;
		default:
				m_common.btf = 1;
	}

	m_varyingColumns.clear();
	m_varyingRows.clear();
}

KLUAdapter::KLUAdapter()
{
    klu_defaults(&m_common);

	// NOTE: klu_defaults should already set the preordering methods correctly.
	// It is repeated here in case this is altered in SuiteSparse at some point

	m_common.scale = 1;
	m_preordering = AMD_ORDERING;
	m_common.btf = 1;

	m_varyingColumns.clear();
	m_varyingRows.clear();
}

void KLUAdapter::preprocessing(SparseMatrix &systemMatrix,
                               std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries)
{
    if (m_symbolic)
    {
        klu_free_symbolic(&m_symbolic, &m_common);
    }

    const Int n = Eigen::internal::convert_index<Int>(systemMatrix.rows());

    auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
    auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());

	m_varyingColumns.clear();
	m_varyingRows.clear();

    m_changedEntries = listVariableSystemMatrixEntries;
    Int varying_entries = Eigen::internal::convert_index<Int>(m_changedEntries.size());

    for (auto &changedEntry : m_changedEntries)
    {
        m_varyingRows.push_back(changedEntry.first);
        m_varyingColumns.push_back(changedEntry.second);
    }

    m_symbolic = klu_analyze_partial(n, Ap, Ai, &m_varyingColumns[0], &m_varyingRows[0], varying_entries, m_preordering, &m_common);

    /* store non-zero value of current preprocessed matrix. only used until
     * to-do in refactorize-function is resolved. Can be removed then. */
    nnz = Eigen::internal::convert_index<Int>(systemMatrix.nonZeros());
}

void KLUAdapter::factorize(SparseMatrix &systemMatrix)
{
    if (m_numeric)
    {
        klu_free_numeric(&m_numeric, &m_common);
    }

    auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
    auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());
    auto Ax = Eigen::internal::convert_index<Real *>(systemMatrix.valuePtr());

    m_numeric = klu_factor(Ap, Ai, Ax, m_symbolic, &m_common);

    /* make sure that factorization path is not computed if there are no varying entries.
     * Doing so should not be a problem, but it is safer to do it this way */
	Int varying_entries = Eigen::internal::convert_index<Int>(m_changedEntries.size());

    if (!(m_varyingColumns.empty()) && !(m_varyingRows.empty()))
    {
        klu_compute_path(m_symbolic, m_numeric, &m_common, Ap, Ai, &m_varyingColumns[0], &m_varyingRows[0], varying_entries);
    }
}

void KLUAdapter::refactorize(SparseMatrix &systemMatrix)
{
    /* TODO: remove if-else when zero<->non-zero issue during matrix stamping has been fixed. Also remove in partialRefactorize then. */
    if (systemMatrix.nonZeros() != nnz)
    {
        preprocessing(systemMatrix, m_changedEntries);
        factorize(systemMatrix);
    }
    else
    {
        auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
        auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());
        auto Ax = Eigen::internal::convert_index<Real *>(systemMatrix.valuePtr());
        klu_refactor(Ap, Ai, Ax, m_symbolic, m_numeric, &m_common);
    }
}

void KLUAdapter::partialRefactorize(SparseMatrix &systemMatrix,
                                    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries)
{
    if (systemMatrix.nonZeros() != nnz)
    {
        preprocessing(systemMatrix, listVariableSystemMatrixEntries);
        factorize(systemMatrix);
    }
    else
    {
        auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
        auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());
        auto Ax = Eigen::internal::convert_index<Real *>(systemMatrix.valuePtr());

        klu_partial_factorization_path(Ap, Ai, Ax, m_symbolic, m_numeric, &m_common);

        if (m_common.status == KLU_PIVOT_FAULT)
        {
            /* pivot became too small => fully factorize again */
            factorize(systemMatrix);
        }
    }
}

Matrix KLUAdapter::solve(Matrix &rightSideVector)
{
    Matrix x = rightSideVector;

    /* number of right hands sides
     * usually one, KLU can handle multiple right hand sides */
    Int rhsCols = Eigen::internal::convert_index<Int>(rightSideVector.cols());

    /* leading dimension, also called "n" */
    Int rhsRows = Eigen::internal::convert_index<Int>(rightSideVector.rows());

	/* tsolve refers to transpose solve. Input matrix is stored in compressed row format,
	 * KLU operates on compressed column format. This way, the transpose of the matrix is factored.
	 * This has to be taken into account only here during right-hand solving. */
    klu_tsolve(m_symbolic, m_numeric, rhsRows, rhsCols, x.const_cast_derived().data(), &m_common);

    return x;
}

void KLUAdapter::printMatrixMarket(SparseMatrix &matrix, int counter) const
{
    std::string outputName = "A" + std::to_string(counter) + ".mtx";
    Int n = Eigen::internal::convert_index<Int>(matrix.rows());

    auto Ap = Eigen::internal::convert_index<const Int *>(matrix.outerIndexPtr());
    auto Ai = Eigen::internal::convert_index<const Int *>(matrix.innerIndexPtr());
    auto Ax = Eigen::internal::convert_index<const Real *>(matrix.valuePtr());
    Int nz = Eigen::internal::convert_index<Int>(matrix.nonZeros());

    std::ofstream ofs;
    ofs.open(outputName);
    /* TODO: add logger to DirectLinearSolver to use libfmt's more powerful logging tools.
	 * Then also move matrix printing (of LU matrices) here in order to avoid C-level printing. */
    ofs.precision(14);
    ofs << "%%MatrixMarket matrix coordinate real general" << std::endl;
    ofs << n << " " << n << " " << nz << std::endl;
    for (int i = 0; i < n; i++)
    {
        for (int j = Ap[i]; j < Ap[i + 1]; j++)
        {
            ofs << i + 1 << " " << Ai[j] + 1 << " " << Ax[j] << std::endl;
        }
    }
    ofs.close();
}
} // namespace DPsim
