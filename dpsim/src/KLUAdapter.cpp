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
        inline int klu_solve(klu_symbolic *Symbolic, klu_numeric *Numeric, Int ldim, Int nrhs, double B [ ], klu_common *Common, double) {
            return klu_solve(Symbolic, Numeric, ldim, nrhs, B, Common);
        }

        inline int klu_solve(klu_symbolic *Symbolic, klu_numeric *Numeric, Int ldim, Int nrhs, std::complex<double> B [ ], klu_common *Common, std::complex<double>) {
            return klu_z_solve(Symbolic, Numeric, ldim, nrhs, &Eigen::numext::real_ref(B[0]), Common);
        }

        inline klu_numeric* klu_factor(Int Ap [ ], Int Ai [ ], Real Ax [ ], klu_symbolic *Symbolic, klu_common *Common, double) {
            return klu_factor(Ap, Ai, Ax, Symbolic, Common);
        }

        inline klu_numeric* klu_factor(Int Ap [ ], Int Ai [ ], std::complex<double> Ax [ ], klu_symbolic *Symbolic, klu_common *Common, std::complex<double>) {
            return klu_z_factor(Ap, Ai, &Eigen::numext::real_ref(Ax[0]), Symbolic, Common);
        }

        inline int klu_refactor(Int Ap [ ], Int Ai [ ], Real Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, double)
        {
            return klu_refactor(Ap, Ai, Ax, Symbolic, Numeric, Common);
        }

        inline int klu_refactor(Int Ap [ ], Int Ai [ ], std::complex<double> Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, std::complex<double>)
        {
            return klu_z_refactor(Ap, Ai, &Eigen::numext::real_ref(Ax[0]), Symbolic, Numeric, Common);
        }

        inline int klu_partial_factorization_path(Int Ap [ ], Int Ai [ ], Real Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, double)
        {
            return klu_partial_factorization_path(Ap, Ai, Ax, Symbolic, Numeric, Common);
        }

        inline int klu_partial_factorization_path(Int Ap [ ], Int Ai [ ], std::complex<double> Ax [ ], klu_symbolic *Symbolic, klu_numeric *Numeric, klu_common *Common, std::complex<double>)
        {
            return klu_z_partial_factorization_path(Ap, Ai, &Eigen::numext::real_ref(Ax[0]), Symbolic, Numeric, Common);
        }

        KLUAdapter::~KLUAdapter()
        {
            if(m_symbolic) klu_free_symbolic(&m_symbolic,&m_common);
            if(m_numeric)  klu_free_numeric(&m_numeric,&m_common);
        }

        void KLUAdapter::initialize()
        {
            klu_defaults(&m_common);
            m_ordering = KLU_AMD_FP;
            m_scaling = 1;
            m_btf = 1;
            m_dump = 0;
            factorization_is_okay = false;
            preprocessing_is_okay = false;

            char* variable = getenv("KLU_SCALING");
            if(variable!=NULL)
            {
                m_scaling = atoi(variable);
                /* m_scaling < 0 valid here (evaluates to no scaling) */
                if(m_scaling > 2)
                {
                    m_scaling = 0;
                }
            }
            variable = getenv("KLU_BTF");
            if(variable!=NULL)
            {
                m_btf = atoi(variable);
                if(m_btf<0 || m_btf>1)
                {
                    m_btf = 1;
                }
            }
            m_common.btf = m_btf;
            m_common.scale = m_scaling;
            variable = getenv("KLU_METHOD");
            if(variable != NULL)
            {
                m_ordering = atoi(variable);
                /* might better be a switch-case? */
                if(m_ordering < KLU_MIN_METHOD || m_ordering>KLU_MAX_METHOD)
                {
                    m_ordering = KLU_AMD_FP;
                }
            }
        }

        void KLUAdapter::preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
        {
            if(m_symbolic)
            {
                preprocessing_is_okay = false;
                klu_free_symbolic(&m_symbolic,&m_common);
            }

            const int n = Eigen::internal::convert_index<int>(mVariableSystemMatrix.rows());

            Int* Ap = const_cast<Int*>(mVariableSystemMatrix.outerIndexPtr());
            Int* Ai = const_cast<Int*>(mVariableSystemMatrix.innerIndexPtr());

            this->changedEntries = mListVariableSystemMatrixEntries;
            int varying_entries = changedEntries.size();
            int varying_columns[varying_entries];
            int varying_rows[varying_entries];

            int index = 0;

            for(std::pair<UInt, UInt> i : changedEntries)
            {
                varying_rows[index] = i.first;
                varying_columns[index] = i.second;
                index++;
            }

            m_symbolic = klu_analyze_partial(n, Ap, Ai, varying_columns, varying_rows, varying_entries, m_ordering, &m_common);

            if(m_symbolic)
            {
                /* successful preordering */
                preprocessing_is_okay = true;
            }

            nnz = mVariableSystemMatrix.nonZeros();
        }

        void KLUAdapter::factorize(SparseMatrix& mVariableSystemMatrix)
        {
            if(m_numeric)
            {
                factorization_is_okay = false;
                klu_free_numeric(&m_numeric,&m_common);
            }

            Int* Ap = const_cast<Int*>(mVariableSystemMatrix.outerIndexPtr());
            Int* Ai = const_cast<Int*>(mVariableSystemMatrix.innerIndexPtr());
            Real* Ax = const_cast<Real*>(mVariableSystemMatrix.valuePtr());
            m_numeric = klu_factor(Ap, Ai, Ax, m_symbolic, &m_common);

            if(m_numeric)
            {
                factorization_is_okay = true;
            }

            int varying_entries = changedEntries.size();
            int varying_columns[varying_entries];
            int varying_rows[varying_entries];
            int index = 0;

            for(std::pair<UInt, UInt> i : changedEntries)
            {
                varying_rows[index] = i.first;
                varying_columns[index] = i.second;
                index++;
            }

            klu_compute_path(m_symbolic, m_numeric, &m_common, Ap, Ai, varying_columns, varying_rows, varying_entries);
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
                Int* Ap = const_cast<Int*>(mVariableSystemMatrix.outerIndexPtr());
                Int* Ai = const_cast<Int*>(mVariableSystemMatrix.innerIndexPtr());
                Real* Ax = const_cast<Real*>(mVariableSystemMatrix.valuePtr());
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
                Int* Ap = const_cast<Int*>(mVariableSystemMatrix.outerIndexPtr());
                Int* Ai = const_cast<Int*>(mVariableSystemMatrix.innerIndexPtr());
                Real* Ax = const_cast<Real*>(mVariableSystemMatrix.valuePtr());
                klu_partial_factorization_path(Ap, Ai, Ax, m_symbolic, m_numeric, &m_common);
                //klu_refactor(Ap, Ai, Ax, m_symbolic, m_numeric, &m_common);

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

        void KLUAdapter::printMTX(const SparseMatrix& matrix, int counter)
        {
            int i, j;
            std::string outputName = "A" + std::to_string(counter) + ".mtx";

            int n = Eigen::internal::convert_index<int>(matrix.rows());

            Int* Ap = const_cast<Int*>(matrix.outerIndexPtr());
            Int* Ai = const_cast<Int*>(matrix.innerIndexPtr());
            Real* Ax = const_cast<Real*>(matrix.valuePtr());
            int nz = Eigen::internal::convert_index<int>(matrix.nonZeros());

            std::ofstream ofs;
            ofs.open(outputName);
            ofs << "%%MatrixMarket matrix coordinate real general" << std::endl;
            ofs << n << " " << n << " " << nz << std::endl;
            for(i = 0 ; i < n ; i++)
            {
                for(j = Ap[i] ; j < Ap[i+1] ; j++)
                {
                    ofs << i+1 << " " << Ai[j]+1 << " " << Ax[j] << std::endl;
                }
            }
            ofs.close();
        }
}
