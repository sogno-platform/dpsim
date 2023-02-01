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
        inline int klu_solve(klu_symbolic *Symbolic, klu_numeric *Numeric, Int ldim, Int nrhs, Real B [ ], klu_common *Common, Real) {
            return klu_solve(Symbolic, Numeric, ldim, nrhs, B, Common);
        }

        inline int klu_solve(klu_symbolic *Symbolic, klu_numeric *Numeric, Int ldim, Int nrhs, std::complex<Real> B [ ], klu_common *Common, std::complex<Real>) {
            return klu_z_solve(Symbolic, Numeric, ldim, nrhs, &Eigen::numext::real_ref(B[0]), Common);
        }

        inline klu_numeric* klu_factor(Int Ap [ ], Int Ai [ ], Real Ax [ ], klu_symbolic *Symbolic, klu_common *Common, Real) {
            return klu_factor(Ap, Ai, Ax, Symbolic, Common);
        }

        inline klu_numeric* klu_factor(Int Ap [ ], Int Ai [ ], std::complex<Real> Ax [ ], klu_symbolic *Symbolic, klu_common *Common, std::complex<Real>) {
            return klu_z_factor(Ap, Ai, &Eigen::numext::real_ref(Ax[0]), Symbolic, Common);
        }

        inline int klu_refactor(Int Ap [ ], Int Ai [ ], Real Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, Real)
        {
            return klu_refactor(Ap, Ai, Ax, Symbolic, Numeric, Common);
        }

        inline int klu_refactor(Int Ap [ ], Int Ai [ ], std::complex<Real> Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, std::complex<Real>)
        {
            return klu_z_refactor(Ap, Ai, &Eigen::numext::real_ref(Ax[0]), Symbolic, Numeric, Common);
        }

        inline int klu_partial_factorization_path(Int Ap [ ], Int Ai [ ], Real Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, Real)
        {
            return klu_partial_factorization_path(Ap, Ai, Ax, Symbolic, Numeric, Common);
        }

        inline int klu_partial_factorization_path(Int Ap [ ], Int Ai [ ], std::complex<Real> Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, std::complex<Real>)
        {
            return klu_z_partial_factorization_path(Ap, Ai, &Eigen::numext::real_ref(Ax[0]), Symbolic, Numeric, Common);
        }

        KLUAdapter::~KLUAdapter()
        {
            if(m_symbolic) klu_free_symbolic(&m_symbolic,&m_common);
            if(m_numeric)  klu_free_numeric(&m_numeric,&m_common);
        }

		KLUAdapter::KLUAdapter() :
                                    m_numeric(nullptr),
                                    m_symbolic(nullptr),
                                    m_ordering(KLU_AMD_FP),
                                    m_btf(1),
                                    m_scaling(1),
                                    factorization_is_okay(false),
                                    preprocessing_is_okay(false)
		{
            klu_defaults(&m_common);

            const char* scaling = std::getenv("KLU_SCALING");
            if(scaling != nullptr)
            {
                m_scaling = atoi(scaling);
                /* m_scaling < 0 valid here (evaluates to no scaling) */
                if(m_scaling > 2)
                {
                    m_scaling = 0;
                }
            }

            const char* do_btf = std::getenv("KLU_BTF");
            if(do_btf != nullptr)
            {
                m_btf = atoi(do_btf);
                if(m_btf<0 || m_btf>1)
                {
                    m_btf = 1;
                }
            }

            m_common.btf = m_btf;
            m_common.scale = m_scaling;

            const char* which_preprocessing_method = std::getenv("KLU_METHOD");
            if(which_preprocessing_method != nullptr)
            {
                m_ordering = atoi(which_preprocessing_method);
                /* might better be a switch-case? */
                if(m_ordering < KLU_MIN_METHOD || m_ordering>KLU_MAX_METHOD)
                {
                    m_ordering = KLU_AMD_FP;
                }
            }
		}

        void KLUAdapter::initialize()
        {
			/* no implementation. initialize() can be removed from DirectLinearSolver.h */
        }

        void KLUAdapter::preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
        {
            if(m_symbolic)
            {
                preprocessing_is_okay = false;
                klu_free_symbolic(&m_symbolic,&m_common);
            }

            const Int n = Eigen::internal::convert_index<Int>(mVariableSystemMatrix.rows());

            auto Ap = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.outerIndexPtr());
            auto Ai = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.innerIndexPtr());

            this->changedEntries = mListVariableSystemMatrixEntries;
            Int varying_entries = Eigen::internal::convert_index<Int>(changedEntries.size());
			std::vector<Int> varying_columns;
			std::vector<Int> varying_rows;

            for(auto i : changedEntries)
            {
                varying_rows.push_back(i.first);
                varying_columns.push_back(i.second);
            }

            m_symbolic = klu_analyze_partial(n, Ap, Ai, &varying_columns[0], &varying_rows[0], varying_entries, m_ordering, &m_common);

            if(m_symbolic)
            {
                /* successful preordering */
                preprocessing_is_okay = true;
            }

            nnz = Eigen::internal::convert_index<Int>(mVariableSystemMatrix.nonZeros());
        }

        void KLUAdapter::factorize(SparseMatrix& mVariableSystemMatrix)
        {
            if(m_numeric)
            {
                factorization_is_okay = false;
                klu_free_numeric(&m_numeric,&m_common);
            }

            auto Ap = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.outerIndexPtr());
            auto Ai = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.innerIndexPtr());
            auto Ax = Eigen::internal::convert_index<Real*>(mVariableSystemMatrix.valuePtr());
            m_numeric = klu_factor(Ap, Ai, Ax, m_symbolic, &m_common);

            if(m_numeric)
            {
                factorization_is_okay = true;
            }

            Int varying_entries = Eigen::internal::convert_index<Int>(changedEntries.size());
			std::vector<Int> varying_columns;
			std::vector<Int> varying_rows;

            for(auto i : changedEntries)
            {
                varying_rows.push_back(i.first);
                varying_columns.push_back(i.second);
            }

            klu_compute_path(m_symbolic, m_numeric, &m_common, Ap, Ai, &varying_columns[0], &varying_rows[0], varying_entries);
        }

        void KLUAdapter::refactorize(SparseMatrix& mVariableSystemMatrix)
        {
            if(mVariableSystemMatrix.nonZeros() != nnz)
            {
                preprocessing(mVariableSystemMatrix, this->changedEntries);
                factorize(mVariableSystemMatrix);
            }
            else
            {
                auto Ap = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.outerIndexPtr());
                auto Ai = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.innerIndexPtr());
                auto Ax = Eigen::internal::convert_index<Real*>(mVariableSystemMatrix.valuePtr());
                klu_refactor(Ap, Ai, Ax, m_symbolic, m_numeric, &m_common);
            }
        }

        void KLUAdapter::partialRefactorize(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
        {
            if(mVariableSystemMatrix.nonZeros() != nnz)
            {
                preprocessing(mVariableSystemMatrix, mListVariableSystemMatrixEntries);
                factorize(mVariableSystemMatrix);
            }
            else
            {
                auto Ap = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.outerIndexPtr());
                auto Ai = Eigen::internal::convert_index<Int*>(mVariableSystemMatrix.innerIndexPtr());
                auto Ax = Eigen::internal::convert_index<Real*>(mVariableSystemMatrix.valuePtr());
                klu_partial_factorization_path(Ap, Ai, Ax, m_symbolic, m_numeric, &m_common, 1.0);

                if(m_common.status == KLU_PIVOT_FAULT)
                {
                    /* pivot became too small => fully factorize again */
                    factorize(mVariableSystemMatrix);
                }
            }
        }

        Matrix KLUAdapter::solve(Matrix& mRightSideVector)
        {
            Matrix x = mRightSideVector;

            /* number of right hands sides
             * usually one, KLU can handle multiple right hand sides */
            Int rhsCols = Eigen::internal::convert_index<Int>(mRightSideVector.cols());

            /* leading dimension, also called "n" */
            Int rhsRows = Eigen::internal::convert_index<Int>(mRightSideVector.rows());

            /*int solve_is_okay = */klu_tsolve(m_symbolic, m_numeric, rhsRows, rhsCols, x.const_cast_derived().data(), const_cast<klu_common*>(&m_common));

            return x;
        }

        void KLUAdapter::printMTX(SparseMatrix& matrix, int counter)
        {
			std::string outputName = "A" + std::to_string(counter) + ".mtx";
            Int n = Eigen::internal::convert_index<Int>(matrix.rows());

            auto Ap = Eigen::internal::convert_index<Int*>(matrix.outerIndexPtr());
            auto Ai = Eigen::internal::convert_index<Int*>(matrix.innerIndexPtr());
            auto Ax = Eigen::internal::convert_index<Real*>(matrix.valuePtr());
            Int nz = Eigen::internal::convert_index<Int>(matrix.nonZeros());

            std::ofstream ofs;
            ofs.open(outputName);
			/* FIXME: determine appropriate precision with respect to datatype chosen (double/float/etc.)
             * Alternatively: add logger to DirectLinearSolver and this type of logging can be done using libfmt.
             * Additionally, the printing of LU/permutation matrices / factorization path / scaling factors / etc.
             * in custom SuiteSparse/KLU can be moved here to reduce the modifications made to SuiteSparse and use
             * C++'s more powerful I/O tools - compared to C-level printing */
			ofs.precision(14);
            ofs << "%%MatrixMarket matrix coordinate real general" << std::endl;
            ofs << n << " " << n << " " << nz << std::endl;
            for(int i = 0 ; i < n ; i++)
            {
                for(int j = Ap[i] ; j < Ap[i+1] ; j++)
                {
                    ofs << i+1 << " " << Ai[j]+1 << " " << Ax[j] << std::endl;
                }
            }
            ofs.close();
        }
}
