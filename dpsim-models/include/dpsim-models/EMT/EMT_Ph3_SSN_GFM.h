// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeVariableSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Three-phase grid-forming inverter represented by a locally
/// linearized affine state-space model.
///
/// The nonlinear model
///
///   x_dot = f(x, u)
///   y     = g(x, u)
///
/// is linearized at the current operating point:
///
///   x_dot = A x + B u + E
///   y     = C x + D u + F
///
/// The terminal convention inherited from
/// TwoTerminalVTypeVariableSSNComp is:
///
///   u = v_terminal1 - v_terminal0
///
/// The SSN output current is defined as current entering the component:
///
///   y = (u - v_c) / R_c
///
/// The physical current injected by the inverter into the network is
/// therefore -y.
class SSN_GFM final : public TwoTerminalVTypeVariableSSNComp,
                      public SharedFactory<SSN_GFM> {

private:
  static constexpr Int mStateSize = 17;
  static constexpr Int mInputSize = 3;
  static constexpr Int mOutputSize = 3;

  enum StateIndex : Int {
    PFiltered = 0,
    QFiltered = 1,
    Omega = 2,
    Theta = 3,
    VoltageMagnitude = 4,

    VoltageIntegratorD = 5,
    VoltageIntegratorQ = 6,

    CurrentIntegratorD = 7,
    CurrentIntegratorQ = 8,

    DelayVoltageD = 9,
    DelayVoltageQ = 10,

    VcA = 11,
    VcB = 12,
    VcC = 13,

    IfA = 14,
    IfB = 15,
    IfC = 16
  };

  // Electrical parameters
  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;

  // Nominal quantities and references
  Real mOmegaN;
  Real mNominalVoltage;
  Real mPRef;
  Real mQRef;

  // VSG parameters
  Real mVirtualInertia;
  Real mDampingCoefficient;
  Real mVoltageDroopGain;
  Real mReactiveIntegralGain;

  // Voltage controller
  Real mKpVoltage;
  Real mKiVoltage;

  // Current controller
  Real mKpCurrent;
  Real mKiCurrent;

  // Active damping and measurement dynamics
  Real mActiveDampingGain;
  Real mPowerFilterCutoff;
  Real mDelayBandwidth;

  // Virtual output impedance (opt-in). Zero recovers the islanded model.
  Real mVirtualResistance;
  Real mVirtualReactance;

  // Scale on the grid-current feedforward in the voltage loop (1 = islanded).
  Real mGridCurrentFeedforward;

  // Proportional Q-V droop (opt-in). Cutoff > 0 selects droop over the integral
  // excitation. Setpoint is captured at initialization.
  Real mReactivePowerDroop;
  Real mReactiveDroopCutoff;
  Real mVoltageSetpoint;

  // Numerical linearization settings
  Real mJacobianRelativeStep;
  Real mJacobianAbsoluteStep;

  // Logging attributes
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;
  const Attribute<Real>::Ptr mOmegaGFM;
  const Attribute<Real>::Ptr mThetaGFM;
  const Attribute<Real>::Ptr mVoltageMagnitudeGFM;

  const Attribute<Real>::Ptr mVcD;
  const Attribute<Real>::Ptr mVcQ;

  const Attribute<Real>::Ptr mIGridD;
  const Attribute<Real>::Ptr mIGridQ;

  const Attribute<Real>::Ptr mIfD;
  const Attribute<Real>::Ptr mIfQ;

  const Attribute<Real>::Ptr mVoltageReferenceD;
  const Attribute<Real>::Ptr mVoltageReferenceQ;

  Matrix getParkTransformMatrix(Real theta) const;

  Matrix getInverseParkTransformMatrix(Real theta) const;

  /// \brief Evaluate the nonlinear state derivative x_dot = f(x,u).
  void evaluateStateDerivative(const Matrix &x, const Matrix &u,
                               Matrix &stateDerivative) const;

  /// \brief Evaluate the nonlinear output y = g(x,u).
  void evaluateOutput(const Matrix &x, const Matrix &u, Matrix &output) const;

  /// \brief Numerically calculate A, B, C and D by central differences.
  void calculateNumericalJacobians(const Matrix &x, const Matrix &u, Matrix &A,
                                   Matrix &B, Matrix &C, Matrix &D) const;

  /// \brief Construct the complete local affine state-space model.
  void buildStateSpaceModel(const Matrix &x, const Matrix &u, Matrix &A,
                            Matrix &B, Matrix &C, Matrix &D, Matrix &E,
                            Matrix &F) const;

  /// \brief Return a safe denominator for the VSG swing equation.
  Real regularizedOmega(Real omega) const;

protected:
  Bool updateComponentParameters() override final;

  void updateLogAttributes(const Matrix &u) const override final;

public:
  using SharedFactory<SSN_GFM>::make;

  SSN_GFM(String uid, String name, Logger::Level logLevel = Logger::Level::off);

  SSN_GFM(String name, Logger::Level logLevel = Logger::Level::off)
      : SSN_GFM(name, name, logLevel) {}

  std::vector<String> getLocalStateNames() const override final;
  Matrix getState() const;
  Matrix getStateDerivative() const;
  Matrix getInterfaceVoltage() const;
  Matrix getInterfaceCurrent() const;
  std::vector<SSNComp::LocalAbcStateBlock>
  getLocalAbcStateBlocks() const override final;

  /// \brief Configure the GFM model.
  ///
  /// nominalVoltage is the nominal line-to-line RMS voltage. With the
  /// power-invariant Park transformation used here, a balanced system's
  /// dq voltage magnitude equals the line-to-line RMS voltage.
  ///
  /// omegaN is the nominal angular frequency in rad/s.
  void setParameters(Real lf, Real cf, Real rf, Real rc, Real nominalVoltage,
                     Real omegaN, Real pRef, Real qRef, Real virtualInertia,
                     Real dampingCoefficient, Real voltageDroopGain,
                     Real reactiveIntegralGain, Real kpVoltage, Real kiVoltage,
                     Real kpCurrent, Real kiCurrent, Real activeDampingGain,
                     Real powerFilterCutoff, Real delayBandwidth);

  /// \brief Configure finite-difference steps for local linearization.
  void setNumericalLinearizationParameters(Real relativeStep,
                                           Real absoluteStep);

  /// \brief Opt-in virtual output impedance Zv = Rv + jXv, subtracted from the
  /// EMF reference across the filter current. Zero (default) is the islanded
  /// model; a finite Rv/Xv damps the power-synchronization loop on a grid.
  void setVirtualImpedance(Real virtualResistance, Real virtualReactance = 0.0);

  /// \brief Scale on the voltage-loop grid-current feedforward (default 1).
  void setGridCurrentFeedforward(Real scale);

  /// \brief Opt-in proportional Q-V droop replacing the integral excitation.
  /// droopGain is Dq [V/var]; cutoff [rad/s] > 0 enables it.
  void setReactivePowerDroop(Real droopGain, Real cutoff);

  void initializeFromNodesAndTerminals(Real frequency) override final;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
