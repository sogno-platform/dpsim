/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/DenseLUAdapter.h>

using namespace DPsim;

namespace DPsim {

DenseLUAdapter::~DenseLUAdapter() = default;

void DenseLUAdapter::preprocessing(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  /* No preprocessing phase needed by PartialPivLU */
}

void DenseLUAdapter::factorize(SparseMatrix &systemMatrix) {
  LUFactorized.compute(Matrix(systemMatrix));
}

void DenseLUAdapter::refactorize(SparseMatrix &systemMatrix) {
  /* only a simple dense factorization */
  LUFactorized.compute(Matrix(systemMatrix));
}

void DenseLUAdapter::partialRefactorize(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  /* only a simple dense factorization */
  LUFactorized.compute(Matrix(systemMatrix));
}

Matrix DenseLUAdapter::solve(Matrix &mRightHandSideVector) {
  return LUFactorized.solve(mRightHandSideVector);
}
} // namespace DPsim
