// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#pragma once

#include <dpsim/Definitions.h>
#include <dpsim/MNAStateSpaceExtractor.h>

#include <vector>

namespace DPsim {

enum class StateSpaceAnalysisFrame {
  Native,
  GlobalDQ0,
};

enum class StateSpacePoleMapping {
  /// Inverse trapezoidal (bilinear) mapping. Use when the complete model was
  /// discretized with the trapezoidal rule and contains no explicit delays.
  Bilinear,

  /// Sampled-data mapping lambda = log(z) / dt. Use for models containing
  /// explicit discrete delays or other non-trapezoidal update operations.
  Logarithmic,
};

/// Performs modal analysis of an extracted discrete-time state-space model.
///
/// The analysis uses the state matrix provided by MNAStateSpaceExtractor and
/// maps discrete-time eigenvalues to continuous-time equivalent eigenvalues
/// using a selectable pole mapping.
class StateSpaceModalAnalysis {
public:
  explicit StateSpaceModalAnalysis(const MNAStateSpaceExtractor &extractor);

  /// Select the coordinate frame used for modal analysis.
  ///
  /// Native uses the extracted state matrix directly.
  /// GlobalDQ0 transforms registered abc state blocks to a global dq0 frame.
  void setAnalysisFrame(StateSpaceAnalysisFrame frame) {
    mAnalysisFrame = frame;
  }

  /// Set the global dq0 frame.
  ///
  /// omega is the constant synchronous angular speed.
  /// theta0 is the global frame angle at t = 0.
  void setGlobalDq0Frame(Real omega, Real theta0 = 0.0) {
    mGlobalOmega = omega;
    mGlobalTheta0 = theta0;
  }

  void setPoleMapping(StateSpacePoleMapping mapping) { mPoleMapping = mapping; }

  /// Update modal quantities from the current extracted state matrix.
  void update();

  /// Eigenvalues of the extracted discrete-time state matrix in the selected analysis frame.
  const CPS::VectorComp &getDiscreteEigenvalues() const {
    return mDiscreteEigenvalues;
  }

  /// Continuous-time equivalent eigenvalues reconstructed with the selected
  /// pole mapping.
  const CPS::VectorComp &getContinuousEigenvalues() const {
    return mContinuousEigenvalues;
  }

  /// Right eigenvectors of the selected discrete analysis state matrix.
  ///
  /// Columns correspond to modes in the same order as getDiscreteEigenvalues().
  /// These eigenvectors represent mode shapes in the selected analysis frame.
  const CPS::MatrixComp &getRightEigenvectors() const {
    return mRightEigenvectors;
  }

  /// Left eigenvectors of the selected discrete analysis state matrix.
  ///
  /// Rows correspond to modes and are normalized such that
  /// getLeftEigenvectors() * getRightEigenvectors() = I.
  const CPS::MatrixComp &getLeftEigenvectors() const {
    return mLeftEigenvectors;
  }

  /// Participation factors of the selected discrete analysis state matrix.
  ///
  /// P(state, mode) = rightEigenvectors(state, mode)
  ///                  * leftEigenvectors(mode, state)
  ///
  /// Rows correspond to states, columns correspond to modes. Columns follow the
  /// same mode order as getDiscreteEigenvalues() and getContinuousEigenvalues().
  const CPS::MatrixComp &getParticipationFactors() const {
    return mParticipationFactors;
  }

  /// State names in the selected analysis frame.
  ///
  /// In Native frame, registered abc states keep their native abc names.
  /// In GlobalDQ0 frame, registered abc state blocks are labelled as dq0 states.
  const std::vector<String> &getStateNames() const { return mStateNames; }

private:
  Matrix buildDiscreteStateMatrixInAnalysisFrame() const;

  Matrix buildGlobalDq0Transformation(Real theta) const;

  std::vector<String> buildStateNamesInAnalysisFrame() const;

  Complex mapDiscreteToContinuous(const Complex &z) const;

  const MNAStateSpaceExtractor &mExtractor;

  StateSpaceAnalysisFrame mAnalysisFrame = StateSpaceAnalysisFrame::Native;

  StateSpacePoleMapping mPoleMapping = StateSpacePoleMapping::Bilinear;

  Real mGlobalOmega = 0.0;

  Real mGlobalTheta0 = 0.0;

  CPS::VectorComp mDiscreteEigenvalues;

  CPS::VectorComp mContinuousEigenvalues;

  CPS::MatrixComp mRightEigenvectors;
  CPS::MatrixComp mLeftEigenvectors;
  CPS::MatrixComp mParticipationFactors;

  std::vector<String> mStateNames;
};

} // namespace DPsim
