// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph1_MixedVTypeVariableSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph1 {

/// Averaged grid-following VSI (PLL + power filter + power/current control,
/// LCL-ish output filter) on MixedVTypeVariableSSNComp: a faithful SSN port of
/// EMT::Ph3::AvVoltSourceInverterStateSpace. 8 real control states (PLL angle
/// deviation from the nominal carrier phase, PLL integrator, P/Q filter,
/// power- and current-control integrators) plus 2 complex envelope states
/// (Vc, If), each individually shifted by -j*omegaN.
class AvVoltSourceInverterStateSpace final
    : public MixedVTypeVariableSSNComp,
      public SharedFactory<AvVoltSourceInverterStateSpace> {
private:
  enum StateIndex : Int {
    Psi = 0,
    PhiPLL = 1,
    PFiltered = 2,
    QFiltered = 3,
    PhiD = 4,
    PhiQ = 5,
    GammaD = 6,
    GammaQ = 7,
    VcRe = 8,
    VcIm = 9,
    IfRe = 10,
    IfIm = 11
  };

  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;

  Real mOmegaN;
  Real mKpPLL;
  Real mKiPLL;

  Real mOmegaCutoff;
  Real mPRef;
  Real mQRef;
  Real mKpPowerCtrl;
  Real mKiPowerCtrl;
  Real mKpCurrCtrl;
  Real mKiCurrCtrl;

  const Attribute<Real>::Ptr mVcD;
  const Attribute<Real>::Ptr mVcQ;
  const Attribute<Real>::Ptr mIrcD;
  const Attribute<Real>::Ptr mIrcQ;
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;
  const Attribute<Real>::Ptr mOmegaPLL;

  /// Builds the affine real model (A,B,C,D,E,F) around (x,u) by evaluating the
  /// nonlinear RHS and its analytic Jacobian, then folding the offset as
  /// E = f(x,u) - A*x - B*u (mirrors ssn-prototype/dp_avvsi_statespace_prototype.py
  /// build_symbolic_model()/eval_model()).
  void buildStateSpaceModel(const Matrix &x, const Matrix &u, Matrix &A,
                            Matrix &B, Matrix &C, Matrix &D, Matrix &E,
                            Matrix &F) const;

protected:
  Bool updateComponentParameters() override final;
  void updateLogAttributes(const Matrix &u) const override final;

public:
  using SharedFactory<AvVoltSourceInverterStateSpace>::make;

  AvVoltSourceInverterStateSpace(String uid, String name,
                                 Logger::Level logLevel = Logger::Level::off);
  AvVoltSourceInverterStateSpace(String name,
                                 Logger::Level logLevel = Logger::Level::off)
      : AvVoltSourceInverterStateSpace(name, name, logLevel) {}

  void setParameters(Real lf, Real cf, Real rf, Real rc, Real omegaN,
                     Real kpPLL, Real kiPLL, Real omegaCutoff, Real pRef,
                     Real qRef, Real kpPowerCtrl, Real kiPowerCtrl,
                     Real kpCurrCtrl, Real kiCurrCtrl);

  void initializeFromNodesAndTerminals(Real frequency) override;
};

} // namespace Ph1
} // namespace DP
} // namespace CPS
