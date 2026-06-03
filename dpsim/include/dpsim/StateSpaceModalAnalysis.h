// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim/Definitions.h>
#include <dpsim/MNAStateSpaceExtractor.h>

namespace DPsim {

/// Performs modal analysis of an extracted discrete-time state-space model.
///
/// The analysis uses the state matrix provided by MNAStateSpaceExtractor and
/// maps discrete-time eigenvalues to continuous-time EMT eigenvalues with the
/// trapezoidal-rule relation:
///
///   lambda = 2 / dt * (z - 1) / (z + 1)
class StateSpaceModalAnalysis {
public:
  explicit StateSpaceModalAnalysis(const MNAStateSpaceExtractor &extractor);

  /// Update modal quantities from the current extracted state matrix.
  void update();

  /// Eigenvalues of the extracted discrete-time state matrix Ad.
  const CPS::VectorComp &getDiscreteEigenvalues() const {
    return mDiscreteEigenvalues;
  }

  /// Continuous-time EMT eigenvalues reconstructed from discrete eigenvalues.
  const CPS::VectorComp &getContinuousEigenvalues() const {
    return mContinuousEigenvalues;
  }

private:
  Complex mapDiscreteToContinuous(const Complex &z) const;

  const MNAStateSpaceExtractor &mExtractor;

  CPS::VectorComp mDiscreteEigenvalues;

  CPS::VectorComp mContinuousEigenvalues;
};

} // namespace DPsim
