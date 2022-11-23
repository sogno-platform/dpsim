/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/SparseLUAdapter.h>

using namespace DPsim;

namespace DPsim
{
    SparseLUAdapter::~SparseLUAdapter(){}

    void SparseLUAdapter::initialize()
    {

    }

    void SparseLUAdapter::preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
    {
        LUFactorizedSparse.analyzePattern(mVariableSystemMatrix);
    }

    void SparseLUAdapter::factorize(SparseMatrix& mVariableSystemMatrix)
    {
        LUFactorizedSparse.factorize(mVariableSystemMatrix);
    }

    void SparseLUAdapter::refactorize(SparseMatrix& mVariableSystemMatrix)
    {
        LUFactorizedSparse.factorize(mVariableSystemMatrix);
    }

    void SparseLUAdapter::partialRefactorize(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
    {
        LUFactorizedSparse.factorize(mVariableSystemMatrix);
    }

    Matrix SparseLUAdapter::solve(Matrix& mRightHandSideVector)
    {
        return LUFactorizedSparse.solve(mRightHandSideVector);
    }
}
