/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/DenseLUAdapter.h>

using namespace DPsim;

namespace DPsim
{

    DenseLUAdapter::~DenseLUAdapter() = default;
    
    void DenseLUAdapter::initialize()
    {
        /* No initialization phase needed by PartialPivLU */
    }
    
    void DenseLUAdapter::preprocessing(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
    {
        /* No preprocessing phase needed by PartialPivLU */
    }
    
    void DenseLUAdapter::factorize(SparseMatrix& mVariableSystemMatrix)
    {
        LUFactorized.compute(Matrix(mVariableSystemMatrix));
    }
    
    void DenseLUAdapter::refactorize(SparseMatrix& mVariableSystemMatrix)
    {
        LUFactorized.compute(Matrix(mVariableSystemMatrix));
    }
    
    void DenseLUAdapter::partialRefactorize(SparseMatrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
    {
        LUFactorized.compute(Matrix(mVariableSystemMatrix));
    }
    
    Matrix DenseLUAdapter::solve(Matrix& mRightHandSideVector)
    {
        return LUFactorized.solve(mRightHandSideVector);
    }
}