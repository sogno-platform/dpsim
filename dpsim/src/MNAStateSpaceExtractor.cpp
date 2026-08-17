// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim/DirectLinearSolver.h>
#include <dpsim/MNAStateSpaceExtractor.h>

#include <stdexcept>
#include <string>

namespace DPsim {

void MNAStateSpaceExtractor::initialize(
    const CPS::MNAInterface::List &components, UInt mnaVectorSize,
    Real timeStep) {
  reset();

  if (timeStep <= 0.0)
    throw std::invalid_argument(
        "MNAStateSpaceExtractor requires a positive time step.");

  mMnaVectorSize = mnaVectorSize;
  mTimeStep = timeStep;

  const auto contributors =
      MNAStateSpaceContributorFactory::createList(components);

  UInt nextStateOffset = 0;

  for (const auto &contributor : contributors) {
    const UInt localStateCount = contributor->getStateCount();

    ContributorEntry entry;
    entry.contributor = contributor;
    entry.stateOffset = nextStateOffset;
    mContributorEntries.push_back(entry);

    nextStateOffset += localStateCount;

    if (contributor->contributesToUpdatedMatrices())
      mHasUpdatedContributors = true;

    const auto dependencies = contributor->getAttributeDependencies();
    mAttributeDependencies.insert(mAttributeDependencies.end(),
                                  dependencies.begin(), dependencies.end());
  }

  mStateCount = nextStateOffset;

  allocateMatrices();
  collectMetadata();

  stampConstantMatrices();
  restampUpdatedMatrices();
  rebuildCombinedMatrices();

  mStateMatrixValid = false;
  mInitialized = true;
}

void MNAStateSpaceExtractor::reset() {
  mInitialized = false;

  mMnaVectorSize = 0;
  mStateCount = 0;
  mTimeStep = 0.0;

  mHasUpdatedContributors = false;
  mStateMatrixValid = false;
  mMetadata = StateSpaceMetadata{};
  mLastExtractionTime = 0.0;
  mHasExtractionTime = false;

  mContributorEntries.clear();
  mAttributeDependencies.clear();

  mAdLocalConstant.resize(0, 0);
  mBdMnaConstant.resize(0, 0);
  mCdMnaConstant.resize(0, 0);

  mAdLocalUpdated.resize(0, 0);
  mBdMnaUpdated.resize(0, 0);
  mCdMnaUpdated.resize(0, 0);

  mAdLocal.resize(0, 0);
  mBdMna.resize(0, 0);
  mCdMna.resize(0, 0);

  mAd.resize(0, 0);
}

void MNAStateSpaceExtractor::extract(DirectLinearSolver &linearSolver,
                                     Bool variableModelChanged,
                                     Bool systemMatrixChanged, Real time) {
  if (!mInitialized)
    throw std::logic_error(
        "MNAStateSpaceExtractor::extract() called before initialize().");

  mLastExtractionTime = time;
  mHasExtractionTime = true;

  if (mStateCount == 0) {
    mStateMatrixValid = true;
    return;
  }

  Bool updatedMatricesChanged = variableModelChanged;

  if (!updatedMatricesChanged) {
    for (const auto &entry : mContributorEntries) {
      if (entry.contributor->contributesToUpdatedMatrices() &&
          entry.contributor->requiresUpdate()) {
        updatedMatricesChanged = true;
        break;
      }
    }
  }

  if (updatedMatricesChanged && mHasUpdatedContributors) {
    restampUpdatedMatrices();
    rebuildCombinedMatrices();
    mStateMatrixValid = false;
  }

  if (systemMatrixChanged)
    mStateMatrixValid = false;

  if (!mStateMatrixValid)
    computeStateMatrix(linearSolver);
}

void MNAStateSpaceExtractor::allocateMatrices() {
  mAdLocalConstant = Matrix::Zero(mStateCount, mStateCount);
  mBdMnaConstant = Matrix::Zero(mStateCount, mMnaVectorSize);
  mCdMnaConstant = Matrix::Zero(mMnaVectorSize, mStateCount);

  mAdLocalUpdated = Matrix::Zero(mStateCount, mStateCount);
  mBdMnaUpdated = Matrix::Zero(mStateCount, mMnaVectorSize);
  mCdMnaUpdated = Matrix::Zero(mMnaVectorSize, mStateCount);

  mAdLocal = Matrix::Zero(mStateCount, mStateCount);
  mBdMna = Matrix::Zero(mStateCount, mMnaVectorSize);
  mCdMna = Matrix::Zero(mMnaVectorSize, mStateCount);

  mAd = Matrix::Zero(mStateCount, mStateCount);
}

void MNAStateSpaceExtractor::collectMetadata() {
  mMetadata = StateSpaceMetadata{};
  mMetadata.stateNames.resize(mStateCount);

  for (const auto &entry : mContributorEntries) {
    entry.contributor->contributeMetadata(mMetadata, entry.stateOffset);
  }

  for (UInt idx = 0; idx < mStateCount; ++idx) {
    if (mMetadata.stateNames[idx].empty())
      mMetadata.stateNames[idx] = "x" + std::to_string(idx);
  }

  for (const auto &abcBlock : mMetadata.abcStateBlocks) {
    if (abcBlock.name.empty()) {
      throw std::runtime_error(
          "MNAStateSpaceExtractor: abc metadata block has an empty name.");
    }

    for (const auto idx : abcBlock.indices) {
      if (idx >= mStateCount) {
        throw std::runtime_error(
            "MNAStateSpaceExtractor: abc metadata index is outside the "
            "extracted state vector.");
      }
    }
  }
}

void MNAStateSpaceExtractor::stampConstantMatrices() {
  mAdLocalConstant.setZero();
  mBdMnaConstant.setZero();
  mCdMnaConstant.setZero();

  for (const auto &entry : mContributorEntries) {
    if (!entry.contributor->contributesToUpdatedMatrices()) {
      entry.contributor->stamp(mAdLocalConstant, mBdMnaConstant, mCdMnaConstant,
                               entry.stateOffset, mMnaVectorSize);
    }
  }
}

void MNAStateSpaceExtractor::restampUpdatedMatrices() {
  mAdLocalUpdated.setZero();
  mBdMnaUpdated.setZero();
  mCdMnaUpdated.setZero();

  for (const auto &entry : mContributorEntries) {
    if (entry.contributor->contributesToUpdatedMatrices()) {
      entry.contributor->stamp(mAdLocalUpdated, mBdMnaUpdated, mCdMnaUpdated,
                               entry.stateOffset, mMnaVectorSize);
    }
  }
}

void MNAStateSpaceExtractor::rebuildCombinedMatrices() {
  mAdLocal = mAdLocalConstant + mAdLocalUpdated;
  mBdMna = mBdMnaConstant + mBdMnaUpdated;
  mCdMna = mCdMnaConstant + mCdMnaUpdated;
}

void MNAStateSpaceExtractor::computeStateMatrix(
    DirectLinearSolver &linearSolver) {
  // DirectLinearSolver::solve takes a non-const Matrix&, so use a local copy.
  Matrix rhs = mCdMna;
  const Matrix mnaToStateSolution = linearSolver.solve(rhs);

  if (mnaToStateSolution.rows() != mMnaVectorSize ||
      mnaToStateSolution.cols() != mStateCount) {
    throw std::runtime_error(
        "MNAStateSpaceExtractor: linear solver returned unexpected "
        "dimensions.");
  }

  mAd = mAdLocal + mBdMna * mnaToStateSolution;

  mStateMatrixValid = true;
}

} // namespace DPsim
