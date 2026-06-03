// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim/DirectLinearSolver.h>
#include <dpsim/MNAStateSpaceExtractor.h>

#include <stdexcept>

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

    if (contributor->isVariable())
      mHasVariableContributors = true;
  }

  mStateCount = nextStateOffset;

  allocateMatrices();

  stampStaticMatrices();
  restampVariableMatrices();
  rebuildCombinedMatrices();

  mStateMatrixValid = false;
  mInitialized = true;
}

void MNAStateSpaceExtractor::reset() {
  mInitialized = false;

  mMnaVectorSize = 0;
  mStateCount = 0;
  mTimeStep = 0.0;

  mHasVariableContributors = false;
  mStateMatrixValid = false;

  mContributorEntries.clear();

  mAdLocalStatic.resize(0, 0);
  mBdMnaStatic.resize(0, 0);
  mCdMnaStatic.resize(0, 0);

  mAdLocalVariable.resize(0, 0);
  mBdMnaVariable.resize(0, 0);
  mCdMnaVariable.resize(0, 0);

  mAdLocal.resize(0, 0);
  mBdMna.resize(0, 0);
  mCdMna.resize(0, 0);

  mAd.resize(0, 0);
}

void MNAStateSpaceExtractor::extract(DirectLinearSolver &linearSolver,
                                     Bool variableModelChanged,
                                     Bool systemMatrixChanged) {
  if (!mInitialized)
    throw std::logic_error(
        "MNAStateSpaceExtractor::extract() called before initialize().");

  if (mStateCount == 0) {
    mStateMatrixValid = true;
    return;
  }

  if (variableModelChanged && mHasVariableContributors) {
    restampVariableMatrices();
    rebuildCombinedMatrices();
    mStateMatrixValid = false;
  }

  if (systemMatrixChanged)
    mStateMatrixValid = false;

  if (!mStateMatrixValid)
    computeStateMatrix(linearSolver);
}

void MNAStateSpaceExtractor::allocateMatrices() {
  mAdLocalStatic = Matrix::Zero(mStateCount, mStateCount);
  mBdMnaStatic = Matrix::Zero(mStateCount, mMnaVectorSize);
  mCdMnaStatic = Matrix::Zero(mMnaVectorSize, mStateCount);

  mAdLocalVariable = Matrix::Zero(mStateCount, mStateCount);
  mBdMnaVariable = Matrix::Zero(mStateCount, mMnaVectorSize);
  mCdMnaVariable = Matrix::Zero(mMnaVectorSize, mStateCount);

  mAdLocal = Matrix::Zero(mStateCount, mStateCount);
  mBdMna = Matrix::Zero(mStateCount, mMnaVectorSize);
  mCdMna = Matrix::Zero(mMnaVectorSize, mStateCount);

  mAd = Matrix::Zero(mStateCount, mStateCount);
}

void MNAStateSpaceExtractor::stampStaticMatrices() {
  mAdLocalStatic.setZero();
  mBdMnaStatic.setZero();
  mCdMnaStatic.setZero();

  for (const auto &entry : mContributorEntries) {
    if (!entry.contributor->isVariable()) {
      entry.contributor->stamp(mAdLocalStatic, mBdMnaStatic, mCdMnaStatic,
                               entry.stateOffset, mMnaVectorSize);
    }
  }
}

void MNAStateSpaceExtractor::restampVariableMatrices() {
  mAdLocalVariable.setZero();
  mBdMnaVariable.setZero();
  mCdMnaVariable.setZero();

  for (const auto &entry : mContributorEntries) {
    if (entry.contributor->isVariable()) {
      entry.contributor->stamp(mAdLocalVariable, mBdMnaVariable, mCdMnaVariable,
                               entry.stateOffset, mMnaVectorSize);
    }
  }
}

void MNAStateSpaceExtractor::rebuildCombinedMatrices() {
  mAdLocal = mAdLocalStatic + mAdLocalVariable;
  mBdMna = mBdMnaStatic + mBdMnaVariable;
  mCdMna = mCdMnaStatic + mCdMnaVariable;
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
