/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/SparseLUAdapter.h>

using namespace DPsim;

namespace DPsim {
SparseLUAdapter::~SparseLUAdapter() = default;

void SparseLUAdapter::preprocessing(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  LUFactorizedSparse.analyzePattern(systemMatrix);
}

void SparseLUAdapter::factorize(SparseMatrix &systemMatrix) {
  LUFactorizedSparse.factorize(systemMatrix);
}

void SparseLUAdapter::refactorize(SparseMatrix &systemMatrix) {
  /* Eigen's SparseLU does not use refactorization. Use regular factorization (numerical factorization and partial pivoting) here */
  LUFactorizedSparse.factorize(systemMatrix);
}

void SparseLUAdapter::partialRefactorize(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  /* Eigen's SparseLU does not use refactorization. Use regular factorization (numerical factorization and partial pivoting) here */
  LUFactorizedSparse.factorize(systemMatrix);
}

Matrix SparseLUAdapter::solve(Matrix &mRightHandSideVector) {
  return LUFactorizedSparse.solve(mRightHandSideVector);
}
} // namespace DPsim
