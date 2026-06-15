// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <memory>
#include <vector>

#include <dpsim/Definitions.h>
#include <dpsim/MNAStateSpaceContributor.h>

namespace DPsim {

class DirectLinearSolver;

/// Extracts the discrete-time state matrix implied by a real-valued EMT MNA
/// model.
///
/// The extractor assembles the MNA-coupled state-space model
///
///   x[k+1]       = AdLocal x[k] + BdMna xMNA[k+1]
///   Y xMNA[k+1] = CdMna x[k]
///
/// where x is the extraction-state vector and xMNA is the full MNA unknown
/// vector. Eliminating xMNA gives
///
///   Ad = AdLocal + BdMna * solve(Y, CdMna)
class MNAStateSpaceExtractor {
public:
  using Ptr = std::shared_ptr<MNAStateSpaceExtractor>;

  MNAStateSpaceExtractor() = default;

  void initialize(const CPS::MNAInterface::List &components, UInt mnaVectorSize,
                  Real timeStep);

  void extract(DirectLinearSolver &linearSolver, Bool variableModelChanged,
               Bool systemMatrixChanged, Real time);

  Bool isInitialized() const { return mInitialized; }

  UInt getStateCount() const { return mStateCount; }

  Real getTimeStep() const { return mTimeStep; }

  const Matrix &getDiscreteStateMatrix() const { return mAd; }

  const StateSpaceMetadata &getMetadata() const { return mMetadata; }

  Bool hasExtractionTime() const { return mHasExtractionTime; }

  Real getLastExtractionTime() const { return mLastExtractionTime; }

private:
  struct ContributorEntry {
    MNAStateSpaceContributor::Ptr contributor;
    UInt stateOffset = 0;
  };

  void reset();

  void allocateMatrices();

  void collectMetadata();

  void stampStaticMatrices();

  void restampVariableMatrices();

  void rebuildCombinedMatrices();

  void computeStateMatrix(DirectLinearSolver &linearSolver);

  Bool mInitialized = false;

  UInt mMnaVectorSize = 0;

  UInt mStateCount = 0;

  Real mTimeStep = 0.0;

  Bool mHasVariableContributors = false;

  Bool mStateMatrixValid = false;

  StateSpaceMetadata mMetadata;

  Real mLastExtractionTime = 0.0;

  Bool mHasExtractionTime = false;

  std::vector<ContributorEntry> mContributorEntries;

  Matrix mAdLocalStatic;
  Matrix mBdMnaStatic;
  Matrix mCdMnaStatic;

  Matrix mAdLocalVariable;
  Matrix mBdMnaVariable;
  Matrix mCdMnaVariable;

  Matrix mAdLocal;
  Matrix mBdMna;
  Matrix mCdMna;

  Matrix mAd;
};

} // namespace DPsim
