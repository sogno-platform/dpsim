/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <array>

#include <dpsim-models/Base/Base_Ph3_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Three-phase EMT switch / circuit breaker.
///
/// Switching modes:
///
/// - Ideal:
///   Conventional binary DPsim switch. All three phases open immediately.
///
/// - CurrentZero:
///   An opening command is armed and each physical pole remains conducting
///   until its own instantaneous EMT phase current reaches/crosses zero.
///
/// - ExponentialZCSEmulation:
///   The phase resistance is increased continuously from R_closed to R_open
///   according to an exponential trajectory over a prescribed switching time:
///
///       R(t) = R_closed * (R_open / R_closed)^alpha
///
///   with alpha in [0,1]. This reproduces the variable-resistance ZCS
///   emulation described by Wirtz et al. and is not an arc model.
class Switch : public MNASimPowerComp<Real>,
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

  /// Opening request is active while a CurrentZero or exponential opening
  /// process has not yet completed.
  const Attribute<Bool>::Ptr mOpeningRequested;

  /// Physical pole states.
  const Attribute<Bool>::Ptr mPoleClosedA;
  const Attribute<Bool>::Ptr mPoleClosedB;
  const Attribute<Bool>::Ptr mPoleClosedC;

  /// Estimated physical zero-crossing times [s], -1 if none has been detected.
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

  Switch(String uid, String name, Logger::Level logLevel = Logger::Level::off);

  Switch(String name, Logger::Level logLevel = Logger::Level::off)
      : Switch(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  /// Select switching model. Configure before simulation initialization.
  void setSwitchingMode(SwitchingMode mode);
  SwitchingMode switchingMode() const { return mSwitchingMode; }

  /// Absolute current tolerance used by CurrentZero mode [A].
  void setZeroCrossingTolerance(Real tolerance);
  Real zeroCrossingTolerance() const { return mZeroCrossingTolerance; }

  /// Set total transition time for ExponentialZCSEmulation [s].
  void setExponentialSwitchingTime(Real switchingTime);
  Real exponentialSwitchingTime() const { return mExponentialSwitchingTime; }

  /// Open command.
  ///
  /// Ideal:
  ///   all poles open immediately.
  ///
  /// CurrentZero:
  ///   wait for individual phase-current zeros.
  ///
  /// ExponentialZCSEmulation:
  ///   arm a simultaneous three-phase exponential resistance increase.
  void openSwitch() override;

  /// Close command.
  ///
  /// Ideal and CurrentZero:
  ///   all poles close immediately. There is no current zero to wait for when
  ///   energising a branch.
  ///
  /// ExponentialZCSEmulation:
  ///   arm a simultaneous three-phase exponential resistance decrease.
  void closeSwitch() override;

  // #### General ####
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### General MNA section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;

  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;

  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  // #### MNA switch interface ####
  Bool mnaIsClosed() override;

  /// Only Ideal is representable by the conventional two precomputed matrices.
  Bool supportsPrecomputedSystemMatrices() const override;

  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx) override;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  // #### Variable component interface ####
  Bool hasParameterChanged() override;

private:
  SwitchingMode mSwitchingMode = SwitchingMode::Ideal;

  Real mZeroCrossingTolerance = 1e-6;

  /// Default matches the 10 ms switching duration used in the benchmark of
  /// the referenced DP ZCS-emulation paper.
  Real mExponentialSwitchingTime = 0.01;

  /// Simulation timestep stored so post-step can prepare R(t+dt), which is
  /// the conductance required by the next MNA solve.
  Real mTimeStep = 0.0;

  std::array<Bool, 3> mPoleClosedPrev{{false, false, false}};
  std::array<Real, 3> mEffectiveResistancePrev{{0.0, 0.0, 0.0}};

  Matrix mPreviousCurrent = Matrix::Zero(3, 1);
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

  MatrixFixedSize<3, 3> currentConductanceMatrix() const;

  Bool currentCrossedZero(Real previousCurrent, Real current) const;

  Real interpolateZeroCrossingTime(Real previousCurrent, Real current,
                                   Real previousTime, Real currentTime) const;

  void setZeroCrossingTime(UInt phase, Real time);
  void updateZeroCrossingState(Real time);
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
