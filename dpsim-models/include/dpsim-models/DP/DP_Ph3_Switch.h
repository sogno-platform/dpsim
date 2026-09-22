// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <array>

#include <dpsim-models/Base/Base_Ph3_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// Dynamic-phasor three-phase breaker.
///
/// Switching modes:
///
/// - Ideal:
///   conventional binary switch.
///
/// - CurrentZero:
///   reconstruct the physical instantaneous phase currents from the phase-peak
///   DP envelopes,
///
///       i_k(t) = Re{ I_k(t) exp(j omega_s t) },
///
///   and open each physical pole at its own current zero.
///
/// - ExponentialZCSEmulation:
///   increase the switch resistance continuously according to
///
///       R(t) = R_closed * (R_open/R_closed)^alpha,
///
///   alpha in [0,1], over a user-defined switching duration. Opening runs
///   alpha from 0 to 1, closing runs it from 1 to 0, since the envelope
///   cannot represent a discontinuity in either direction. This is the
///   variable-resistance ZCS-emulation approach and is not an arc model.
class Switch : public MNASimPowerComp<Complex>,
               public Base::Ph3::Switch,
               public SharedFactory<Switch>,
               public MNAVariableCompInterface,
               public MNASwitchInterface {
public:
  enum class SwitchingMode {
    Ideal = 0,
    CurrentZero = 1,
    ExponentialZCSEmulation = 2,
  };

  const Attribute<Bool>::Ptr mOpeningRequested;

  const Attribute<Bool>::Ptr mPoleClosedA;
  const Attribute<Bool>::Ptr mPoleClosedB;
  const Attribute<Bool>::Ptr mPoleClosedC;

  /// Reconstructed physical instantaneous currents [A].
  const Attribute<Real>::Ptr mInstantCurrentA;
  const Attribute<Real>::Ptr mInstantCurrentB;
  const Attribute<Real>::Ptr mInstantCurrentC;

  const Attribute<Real>::Ptr mZeroCrossingTimeA;
  const Attribute<Real>::Ptr mZeroCrossingTimeB;
  const Attribute<Real>::Ptr mZeroCrossingTimeC;

  /// Exponential-transition diagnostics.
  const Attribute<Bool>::Ptr mExponentialTransitionActive;
  /// True while the active exponential transition is a closing transition.
  const Attribute<Bool>::Ptr mExponentialTransitionClosing;
  /// Position on the resistance path, 0 at R_closed and 1 at R_open. Runs
  /// 0 -> 1 while opening and 1 -> 0 while closing.
  const Attribute<Real>::Ptr mExponentialProgress;
  const Attribute<Real>::Ptr mExponentialTransitionStartTime;
  const Attribute<Real>::Ptr mExponentialTransitionEndTime;

  /// Effective phase resistances currently stamped into the MNA matrix [ohm].
  const Attribute<Real>::Ptr mEffectiveResistanceA;
  const Attribute<Real>::Ptr mEffectiveResistanceB;
  const Attribute<Real>::Ptr mEffectiveResistanceC;

  Switch(String uid, String name, Logger::Level loglevel = Logger::Level::off);

  Switch(String name, Logger::Level logLevel = Logger::Level::off)
      : Switch(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  void setSwitchingMode(SwitchingMode mode);
  SwitchingMode switchingMode() const { return mSwitchingMode; }

  void setZeroCrossingTolerance(Real tolerance);

  void setExponentialSwitchingTime(Real switchingTime);
  Real exponentialSwitchingTime() const { return mExponentialSwitchingTime; }

  void openSwitch() override;
  void closeSwitch() override;

  // #### General ####
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### General MNA ####
  /// mTimeStep is the lookahead the exponential ramp is evaluated at.
  bool mnaUpdateTimeStep(Real timeStep) override {
    if (timeStep <= 0)
      return false;

    mTimeStep = timeStep;
    return true;
  }
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;

  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;

  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  // #### Switch interface ####
  Bool mnaIsClosed() override;

  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx) override;

  Bool supportsPrecomputedSystemMatrices() const override;

  // #### Variable-component interface ####
  Bool hasParameterChanged() override;

private:
  SwitchingMode mSwitchingMode = SwitchingMode::Ideal;

  Real mZeroCrossingTolerance = 1e-6;
  Real mExponentialSwitchingTime = 0.01;
  Real mTimeStep = 0.0;

  /// Reference-frame angular frequency supplied by the solver [rad/s].
  Real mShiftOmega = 0.0;

  std::array<Bool, 3> mPoleClosedPrev{{false, false, false}};
  std::array<Real, 3> mEffectiveResistancePrev{{0.0, 0.0, 0.0}};

  Matrix mPreviousInstantaneousCurrent = Matrix::Zero(3, 1);
  Real mPreviousCurrentTime = 0.0;
  Bool mPreviousCurrentValid = false;

  Bool mResetZeroCrossingHistory = false;
  Bool mExponentialTransitionStarted = false;

  Bool poleClosed(UInt phase) const;
  void setPoleClosed(UInt phase, Bool closed);
  void setAllPoles(Bool closed);

  Bool allPolesClosed() const;
  Bool allPolesOpen() const;

  void resetZeroCrossingTimes();

  Real effectiveResistance(UInt phase) const;
  void setEffectiveResistance(UInt phase, Real resistance);
  void synchronizeEffectiveResistance(Bool closed);

  void resetExponentialTransition();
  void validateExponentialResistanceParameters() const;
  Real exponentialResistance(UInt phase, Real alpha) const;
  void updateExponentialTransition(Real time);

  MatrixFixedSizeComp<3, 3> currentConductanceMatrix() const;

  Matrix reconstructInstantaneousCurrent(Real time) const;
  void setInstantaneousCurrentAttributes(const Matrix &current);

  Bool currentCrossedZero(Real previousCurrent, Real current) const;

  Real interpolateZeroCrossingTime(Real previousCurrent, Real current,
                                   Real previousTime, Real currentTime) const;

  void setZeroCrossingTime(UInt phase, Real time);
  void updateZeroCrossingState(Real time);
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
