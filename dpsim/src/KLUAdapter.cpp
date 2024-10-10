/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/KLUAdapter.h>

using namespace DPsim;

namespace DPsim {
KLUAdapter::~KLUAdapter() {
  if (mSymbolic)
    klu_free_symbolic(&mSymbolic, &mCommon);
  if (mNumeric)
    klu_free_numeric(&mNumeric, &mCommon);
  SPDLOG_LOGGER_INFO(mSLog, "Number of Pivot Faults: {}", mPivotFaults);
}

KLUAdapter::KLUAdapter() {
  klu_defaults(&mCommon);

  // NOTE: klu_defaults should already set the preordering methods correctly.
  // It is repeated here in case this is altered in SuiteSparse at some point

  mCommon.scale = 2;
  mPreordering = AMD_ORDERING;
  mCommon.btf = 1;

  mVaryingColumns.clear();
  mVaryingRows.clear();
}

KLUAdapter::KLUAdapter(CPS::Logger::Log log) : KLUAdapter() {
  this->mSLog = log;
}

void KLUAdapter::preprocessing(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  if (mSymbolic) {
    klu_free_symbolic(&mSymbolic, &mCommon);
  }

  const Int n = Eigen::internal::convert_index<Int>(systemMatrix.rows());

  auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
  auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());

  mVaryingColumns.clear();
  mVaryingRows.clear();

  mChangedEntries = listVariableSystemMatrixEntries;
  Int varying_entries =
      Eigen::internal::convert_index<Int>(mChangedEntries.size());

  for (auto &changedEntry : mChangedEntries) {
    mVaryingRows.push_back(changedEntry.first);
    mVaryingColumns.push_back(changedEntry.second);
  }

  // this call also works if mVaryingColumns, mVaryingRows are empty
  mSymbolic =
      klu_analyze_partial(n, Ap, Ai, &mVaryingColumns[0], &mVaryingRows[0],
                          varying_entries, mPreordering, &mCommon);

  /* store non-zero value of current preprocessed matrix. only used until
     * to-do in refactorize-function is resolved. Can be removed then. */
  nnz = Eigen::internal::convert_index<Int>(systemMatrix.nonZeros());
}

void KLUAdapter::factorize(SparseMatrix &systemMatrix) {
  if (mNumeric) {
    klu_free_numeric(&mNumeric, &mCommon);
  }

  auto Ap = Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
  auto Ai = Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());
  auto Ax = Eigen::internal::convert_index<Real *>(systemMatrix.valuePtr());

  mNumeric = klu_factor(Ap, Ai, Ax, mSymbolic, &mCommon);

  Int varying_entries =
      Eigen::internal::convert_index<Int>(mChangedEntries.size());

  /* make sure that factorization path is not computed if there are no varying entries.
     * Doing so should not be a problem, but it is safer to do it this way */
  if (!(mVaryingColumns.empty()) && !(mVaryingRows.empty())) {
    if (mPartialRefactorizationMethod ==
        DPsim::PARTIAL_REFACTORIZATION_METHOD::FACTORIZATION_PATH) {
      klu_compute_path(mSymbolic, mNumeric, &mCommon, Ap, Ai,
                       &mVaryingColumns[0], &mVaryingRows[0], varying_entries);
    } else if (mPartialRefactorizationMethod ==
               DPsim::PARTIAL_REFACTORIZATION_METHOD::REFACTORIZATION_RESTART) {
      klu_determine_start(mSymbolic, mNumeric, &mCommon, Ap, Ai,
                          &mVaryingColumns[0], &mVaryingRows[0],
                          varying_entries);
    }
  }
}

void KLUAdapter::refactorize(SparseMatrix &systemMatrix) {
  /* TODO: remove if-else when zero<->non-zero issue during matrix stamping has been fixed. Also remove in partialRefactorize then. */
  if (systemMatrix.nonZeros() != nnz) {
    preprocessing(systemMatrix, mChangedEntries);
    factorize(systemMatrix);
  } else {
    auto Ap =
        Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
    auto Ai =
        Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());
    auto Ax = Eigen::internal::convert_index<Real *>(systemMatrix.valuePtr());
    klu_refactor(Ap, Ai, Ax, mSymbolic, mNumeric, &mCommon);
  }
}

void KLUAdapter::partialRefactorize(
    SparseMatrix &systemMatrix,
    std::vector<std::pair<UInt, UInt>> &listVariableSystemMatrixEntries) {
  if (systemMatrix.nonZeros() != nnz) {
    preprocessing(systemMatrix, listVariableSystemMatrixEntries);
    factorize(systemMatrix);
  } else {
    auto Ap =
        Eigen::internal::convert_index<Int *>(systemMatrix.outerIndexPtr());
    auto Ai =
        Eigen::internal::convert_index<Int *>(systemMatrix.innerIndexPtr());
    auto Ax = Eigen::internal::convert_index<Real *>(systemMatrix.valuePtr());

    if (mPartialRefactorizationMethod ==
        PARTIAL_REFACTORIZATION_METHOD::FACTORIZATION_PATH) {
      klu_partial_factorization_path(Ap, Ai, Ax, mSymbolic, mNumeric, &mCommon);
    } else if (mPartialRefactorizationMethod ==
               PARTIAL_REFACTORIZATION_METHOD::REFACTORIZATION_RESTART) {
      klu_partial_refactorization_restart(Ap, Ai, Ax, mSymbolic, mNumeric,
                                          &mCommon);
    } else {
      klu_refactor(Ap, Ai, Ax, mSymbolic, mNumeric, &mCommon);
    }

    if (mCommon.status == KLU_PIVOT_FAULT) {
      /* pivot became too small => fully factorize again */
      mPivotFaults++;
      factorize(systemMatrix);
    }
  }
}

Matrix KLUAdapter::solve(Matrix &rightSideVector) {
  // TODO: this calls malloc, which is not allowed in the simulation loop
  // We should preallocate this buffer.
  Matrix x = rightSideVector;

  /* number of right hands sides
     * usually one, KLU can handle multiple right hand sides */
  Int rhsCols = Eigen::internal::convert_index<Int>(rightSideVector.cols());

  /* leading dimension, also called "n" */
  Int rhsRows = Eigen::internal::convert_index<Int>(rightSideVector.rows());

  /* tsolve refers to transpose solve. Input matrix is stored in compressed row format,
	 * KLU operates on compressed column format. This way, the transpose of the matrix is factored.
	 * This has to be taken into account only here during right-hand solving. */
  klu_tsolve(mSymbolic, mNumeric, rhsRows, rhsCols,
             x.const_cast_derived().data(), &mCommon);

  return x;
}

void KLUAdapter::printMatrixMarket(SparseMatrix &matrix, int counter) const {
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
  for (int i = 0; i < n; i++) {
    for (int j = Ap[i]; j < Ap[i + 1]; j++) {
      ofs << i + 1 << " " << Ai[j] + 1 << " " << Ax[j] << std::endl;
    }
  }
  ofs.close();
}

void KLUAdapter::applyConfiguration() {
  switch (mConfiguration.getScalingMethod()) {
  case SCALING_METHOD::NO_SCALING:
    mCommon.scale = 0;
    break;
  case SCALING_METHOD::SUM_SCALING:
    mCommon.scale = 1;
    break;
  case SCALING_METHOD::MAX_SCALING:
    mCommon.scale = 2;
    break;
  default:
    mCommon.scale = 1;
  }

  SPDLOG_LOGGER_INFO(mSLog, "Matrix is scaled using " +
                                mConfiguration.getScalingMethodString());

  // TODO: implement support for COLAMD (modifiy SuiteSparse)
  switch (mConfiguration.getFillInReductionMethod()) {
  case FILL_IN_REDUCTION_METHOD::AMD:
    mPreordering = AMD_ORDERING;
    break;
  case FILL_IN_REDUCTION_METHOD::AMD_NV:
    mPreordering = AMD_ORDERING_NV;
    break;
  case FILL_IN_REDUCTION_METHOD::AMD_RA:
    mPreordering = AMD_ORDERING_RA;
    break;
  default:
    mPreordering = AMD_ORDERING;
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "Matrix is fill reduced with " +
                         mConfiguration.getFillInReductionMethodString());

  // NOTE: in case more partial refactorization methods are defined/developed, that are not implemented in KLU, this assigment would be invalid
  mPartialRefactorizationMethod =
      mConfiguration.getPartialRefactorizationMethod();

  SPDLOG_LOGGER_INFO(
      mSLog, "Matrix is refactored " +
                 mConfiguration.getPartialRefactorizationMethodString());

  switch (mConfiguration.getBTF()) {
  case USE_BTF::DO_BTF:
    mCommon.btf = 1;
    break;
  case USE_BTF::NO_BTF:
    mCommon.btf = 0;
    break;
  default:
    mCommon.btf = 1;
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "Matrix is permuted " + mConfiguration.getBTFString());
}
} // namespace DPsim
