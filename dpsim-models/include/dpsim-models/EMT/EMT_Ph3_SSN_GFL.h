// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeVariableSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Three-phase grid-following inverter represented by a locally
/// linearized affine state-space model.
///
/// The nonlinear model
///
///   x_dot = f(x, u)
///   y     = g(x, u)
///
/// is analytically linearized at the current operating point:
///
///   x_dot = A x + B u + E
///   y     = C x + D u + F
///
/// The terminal convention inherited from
/// TwoTerminalVTypeVariableSSNComp is:
///
///   u = v_terminal1 - v_terminal0
///
/// The physical grid current used by the controller is positive for inverter
/// injection:
///
///   i_grid = (v_c - u) / R_c
///
/// The SSN output current is current entering the component and therefore has
/// the opposite sign:
///
///   y = (u - v_c) / R_c
class SSN_GFL final : public TwoTerminalVTypeVariableSSNComp,
                      public SharedFactory<SSN_GFL> {
private:
  static constexpr Int mStateSize = 14;
  static constexpr Int mInputSize = 3;
  static constexpr Int mOutputSize = 3;

  enum StateIndex : Int {
    ThetaPLL = 0,
    PhiPLL = 1,
    PFiltered = 2,
    QFiltered = 3,
    PhiD = 4,
    PhiQ = 5,
    GammaD = 6,
    GammaQ = 7,
    VcA = 8,
    VcB = 9,
    VcC = 10,
    IfA = 11,
    IfB = 12,
    IfC = 13
  };

  // Electrical parameters
  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;

  // PLL parameters
  Real mOmegaN;
  Real mKpPLL;
  Real mKiPLL;

  // Power measurement and controller parameters
  Real mOmegaCutoff;
  Real mPRef;
  Real mQRef;
  Real mKpPowerCtrl;
  Real mKiPowerCtrl;
  Real mKpCurrCtrl;
  Real mKiCurrCtrl;

  // Logging attributes
  const Attribute<Real>::Ptr mVcD;
  const Attribute<Real>::Ptr mVcQ;
  const Attribute<Real>::Ptr mIrcD;
  const Attribute<Real>::Ptr mIrcQ;
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;
  const Attribute<Real>::Ptr mOmegaPLL;

  Matrix getParkTransformMatrix(Real theta) const;
  Matrix getInverseParkTransformMatrix(Real theta) const;

  /// \brief Evaluate the nonlinear state derivative x_dot = f(x,u).
  void evaluateStateDerivative(const Matrix &x, const Matrix &u,
                               Matrix &stateDerivative) const;

  /// \brief Evaluate the nonlinear SSN output y = g(x,u).
  void evaluateOutput(const Matrix &x, const Matrix &u, Matrix &output) const;

  /// \brief Calculate the exact local Jacobians A, B, C and D.
  void calculateAnalyticalJacobians(const Matrix &x, const Matrix &u, Matrix &A,
                                    Matrix &B, Matrix &C, Matrix &D) const;

  /// \brief Construct the complete local affine state-space model.
  void buildStateSpaceModel(const Matrix &x, const Matrix &u, Matrix &A,
                            Matrix &B, Matrix &C, Matrix &D, Matrix &E,
                            Matrix &F) const;

protected:
  Bool updateComponentParameters() override final;
  void updateLogAttributes(const Matrix &u) const override final;

public:
  using SharedFactory<SSN_GFL>::make;

  SSN_GFL(String uid, String name, Logger::Level logLevel = Logger::Level::off);

  SSN_GFL(String name, Logger::Level logLevel = Logger::Level::off)
      : SSN_GFL(name, name, logLevel) {}

  std::vector<String> getLocalStateNames() const override final;

  std::vector<SSNComp::LocalAbcStateBlock>
  getLocalAbcStateBlocks() const override final;

  /// \brief Configure the GFL filter, PLL, power loop and current loop.
  ///
  /// This signature intentionally matches the former
  /// AvVoltSourceInverterStateSpace model so existing parameter sets can be
  /// transferred without conversion.
  void setParameters(Real lf, Real cf, Real rf, Real rc, Real omegaN,
                     Real kpPLL, Real kiPLL, Real omegaCutoff, Real pRef,
                     Real qRef, Real kpPowerCtrl, Real kiPowerCtrl,
                     Real kpCurrCtrl, Real kiCurrCtrl);

  void initializeFromNodesAndTerminals(Real frequency) override final;

  Matrix getState() const;
  Matrix getStateDerivative() const;
  Matrix getInterfaceVoltage() const;
  Matrix getInterfaceCurrent() const;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
