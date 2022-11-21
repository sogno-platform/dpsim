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

    DenseLUAdapter::~DenseLUAdapter(){}
    
    void DenseLUAdapter::initialize()
    {

    }
    
    void DenseLUAdapter::preprocessing(Matrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
    {
        
    }
    
    void DenseLUAdapter::factorize(Matrix& mVariableSystemMatrix)
    {
        LUFactorized.compute(mVariableSystemMatrix);
    }
    
    void DenseLUAdapter::refactorize(Matrix& mVariableSystemMatrix)
    {
        LUFactorized.compute(mVariableSystemMatrix);
    }
    
    void DenseLUAdapter::partialRefactorize(Matrix& mVariableSystemMatrix, std::vector<std::pair<UInt, UInt>>& mListVariableSystemMatrixEntries)
    {
        LUFactorized.compute(mVariableSystemMatrix);
    }
    
    Matrix DenseLUAdapter::solve(Matrix& mRightHandSideVector)
    {
        return LUFactorized.solve(mRightHandSideVector);
    }
}