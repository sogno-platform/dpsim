/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Dynamic phasor switch
///
/// The switch can be opened and closed.
/// Each state has a specific resistance value.
///
/// Switching modes:
///
/// - Ideal:
///   conventional binary switch.
///
/// - CurrentZero:
///   reconstruct the physical instantaneous current from the DP envelope,
///
///       i(t) = Re{ I(t) exp(j omega_s t) },
///
///   and open at its current zero.
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
               public Base::Ph1::Switch,
               public SharedFactory<Switch>,
               public MNASwitchInterface,
               public MNAVariableCompInterface {
public:
  enum class SwitchingMode {
    Ideal = 0,
    CurrentZero = 1,
    ExponentialZCSEmulation = 2,
  };

  const Attribute<Bool>::Ptr mOpeningRequested;

  const Attribute<Bool>::Ptr mPoleClosed;

  /// Reconstructed physical instantaneous current [A].
  const Attribute<Real>::Ptr mInstantCurrent;

  const Attribute<Real>::Ptr mZeroCrossingTime;

  /// Exponential-transition diagnostics.
  const Attribute<Bool>::Ptr mExponentialTransitionActive;
  /// True while the active exponential transition is a closing transition.
  const Attribute<Bool>::Ptr mExponentialTransitionClosing;
  /// Position on the resistance path, 0 at R_closed and 1 at R_open. Runs
  /// 0 -> 1 while opening and 1 -> 0 while closing.
  const Attribute<Real>::Ptr mExponentialProgress;
  const Attribute<Real>::Ptr mExponentialTransitionStartTime;
  const Attribute<Real>::Ptr mExponentialTransitionEndTime;

  /// Effective resistance currently stamped into the MNA matrix [ohm].
  const Attribute<Real>::Ptr mEffectiveResistance;

  /// Defines UID, name, component parameters and logging level
  Switch(String uid, String name, Logger::Level loglevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
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
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### General MNA section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  // #### MNA section for switch ####
  /// Check if switch is closed
  Bool mnaIsClosed() override;
  /// Stamps system matrix considering the defined switch position
  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx) override;

  Bool supportsPrecomputedSystemMatrices() const override;

  // #### MNA section for variable component ####
  Bool hasParameterChanged() override;

private:
  SwitchingMode mSwitchingMode = SwitchingMode::Ideal;

  Real mZeroCrossingTolerance = 1e-6;
  Real mExponentialSwitchingTime = 0.01;
  Real mTimeStep = 0.0;

  /// Reference-frame angular frequency supplied by the solver [rad/s].
  Real mShiftOmega = 0.0;

  Bool mPoleClosedPrev = false;
  Real mEffectiveResistancePrev = 0.0;

  Real mPreviousInstantaneousCurrent = 0.0;
  Real mPreviousCurrentTime = 0.0;
  Bool mPreviousCurrentValid = false;

  Bool mResetZeroCrossingHistory = false;
  Bool mExponentialTransitionStarted = false;

  void setPoleClosed(Bool closed);

  void resetZeroCrossingTime();

  void synchronizeEffectiveResistance(Bool closed);

  void resetExponentialTransition();
  void validateExponentialResistanceParameters() const;
  Real exponentialResistance(Real alpha) const;
  void updateExponentialTransition(Real time);

  Complex currentConductance() const;

  Real reconstructInstantaneousCurrent(Real time) const;

  Bool currentCrossedZero(Real previousCurrent, Real current) const;

  Real interpolateZeroCrossingTime(Real previousCurrent, Real current,
                                   Real previousTime, Real currentTime) const;

  void updateZeroCrossingState(Real time);
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
