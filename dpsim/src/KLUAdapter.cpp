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

KLUAdapter::KLUAdapter()
{
    klu_defaults(&m_common);

    if (const char *scaling = std::getenv("KLU_SCALING"))
    {
        m_scaling = atoi(scaling);

        /* m_scaling < 0 valid here (evaluates to "no scaling") */
        if (m_scaling > 2)
        {
            m_scaling = 0;
        }
    }

    if (const char *do_btf = std::getenv("KLU_BTF"))
    {
        if (atoi(do_btf) != 0)
        {
            m_btf = true;
        }
    }

    m_common.btf = m_btf ? 1 : 0;
    m_common.scale = m_scaling;

    if (const char *which_preprocessing_method = std::getenv("KLU_METHOD"))
    {
        m_partial_method = atoi(which_preprocessing_method);

        /* "orderings" internally defined in custom KLU module. See klu.h for available
         * values and settings. Probably a fix-me at some point - rename "method" appropriately.
         * default is KLU_AMD_FP, i.e. KLU+AMD(+factorization path) */

        if (m_partial_method < KLU_MIN_METHOD || m_partial_method > KLU_MAX_METHOD)
        {
            m_partial_method = KLU_AMD_FP;
        }
    }
}

void KLUAdapter::preprocessing(SparseMatrix &systemMatrix,
                               std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries)
{
    if (m_symbolic)
    {
        preprocessing_is_okay = false;
        klu_free_symbolic(&m_symbolic, &m_common);
    }

    const Int n = Eigen::internal::convert_index<Int>(systemMatrix.rows());

    auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
    auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());

    /* TODO: possible improvement: don't save variable system matrix entries, but rather the varying rows/cols vector
     * i.e. do the computations below only once */
    this->changedEntries = listVariableSystemMatrixEntries;
    Int varying_entries = Eigen::internal::convert_index<Int>(changedEntries.size());
    std::vector<Int> varying_columns;
    std::vector<Int> varying_rows;

    for (auto &changedEntry : changedEntries)
    {
        varying_rows.push_back(changedEntry.first);
        varying_columns.push_back(changedEntry.second);
    }

    m_symbolic = klu_analyze_partial(n, Ap, Ai, &varying_columns[0], &varying_rows[0], varying_entries,
                                     m_partial_method, &m_common);

    if (m_symbolic)
    {
        /* successful preordering */
        preprocessing_is_okay = true;
    }

    /* store non-zero value of current preprocessed matrix. only used until
     * to-do in refactorize-function is resolved. Can be removed then. */
    nnz = Eigen::internal::convert_index<Int>(systemMatrix.nonZeros());
}

void KLUAdapter::factorize(SparseMatrix &systemMatrix)
{
    if (m_numeric)
    {
        factorization_is_okay = false;
        klu_free_numeric(&m_numeric, &m_common);
    }

    auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
    auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());
    auto Ax = Eigen::internal::convert_index<Real *>(systemMatrix.valuePtr());

    m_numeric = klu_factor(Ap, Ai, Ax, m_symbolic, &m_common);

    if (m_numeric)
    {
        factorization_is_okay = true;
    }

    /* make sure that factorization path is not computed if there are no varying entries.
     * Doing so should not be a problem, but it is safer to do it this way */
    if (!(this->changedEntries.empty()))
    {
        Int varying_entries = Eigen::internal::convert_index<Int>(changedEntries.size());
        std::vector<Int> varying_columns;
        std::vector<Int> varying_rows;

        for (auto &changedEntry : changedEntries)
        {
            varying_rows.push_back(changedEntry.first);
            varying_columns.push_back(changedEntry.second);
        }
        klu_compute_path(m_symbolic, m_numeric, &m_common, Ap, Ai, &varying_columns[0], &varying_rows[0],
                         varying_entries);
    }
}

void KLUAdapter::refactorize(SparseMatrix &systemMatrix)
{
    /* TODO: in DPsim, the non-zero count between the "first" factorization and the first
     * refactorization might vary. This is because during the "stamping" phase of the matrix
     * before preprocessing (see stampVariableSystemMatrix() in MnaSolverDirect), some values
     * of certain components are not set properly, but in recomputeSystemMatrix, they are set.
     * Thus, the matrix pattern changes between first factorization and now, and this function
     * needs to re-do preprocessing and factorization (plus factorization path, if necessary).
     * This needs to be fixed in MnaSolver(Direct), because it might affect performance.
     * Same for partialRefactorize below this function. */
    if (systemMatrix.nonZeros() != nnz)
    {
        preprocessing(systemMatrix, this->changedEntries);
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
    /* TODO: ensure matrix has been factorized properly before calling this function.
     * assertions might hurt performance, thus omitted here */

    Matrix x = rightSideVector;

    /* number of right hands sides
     * usually one, KLU can handle multiple right hand sides */
    Int rhsCols = Eigen::internal::convert_index<Int>(rightSideVector.cols());

    /* leading dimension, also called "n" */
    Int rhsRows = Eigen::internal::convert_index<Int>(rightSideVector.rows());

    klu_tsolve(m_symbolic, m_numeric, rhsRows, rhsCols, x.const_cast_derived().data(),
               const_cast<klu_common *>(&m_common));

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
    /* TODO: determine appropriate precision with respect to datatype chosen (double/float/etc.)
     * Alternatively: add logger to DirectLinearSolver and this type of logging can be done using libfmt.
     * Additionally, the printing of LU/permutation matrices / factorization path / scaling factors / etc.
     * in custom SuiteSparse/KLU can be moved here to reduce the modifications made to SuiteSparse and use
     * C++'s more powerful I/O tools - compared to C-level printing */
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
