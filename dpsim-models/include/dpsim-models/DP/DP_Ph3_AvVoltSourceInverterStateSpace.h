// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph3_MixedVTypeVariableSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// Averaged grid-following VSI SSN port of EMT::Ph3::AvVoltSourceInverterStateSpace: 10 real control states (single PLL, positive-sequence dq frame plus a negative-sequence current-control loop, gamma_nd/gamma_nq) plus 6 complex per-phase envelope states (Vc_a/b/c, If_a/b/c).
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
    GammaND = 8,
    GammaNQ = 9,
    VcARe = 10,
    VcAIm = 11,
    VcBRe = 12,
    VcBIm = 13,
    VcCRe = 14,
    VcCIm = 15,
    IfARe = 16,
    IfAIm = 17,
    IfBRe = 18,
    IfBIm = 19,
    IfCRe = 20,
    IfCIm = 21
  };

  static constexpr Int mVcReCol[3] = {VcARe, VcBRe, VcCRe};
  static constexpr Int mVcImCol[3] = {VcAIm, VcBIm, VcCIm};
  static constexpr Int mIfReCol[3] = {IfARe, IfBRe, IfCRe};
  static constexpr Int mIfImCol[3] = {IfAIm, IfBIm, IfCIm};
  static constexpr Int mUReCol[3] = {0, 2, 4};
  static constexpr Int mUImCol[3] = {1, 3, 5};

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

  /// Negative-sequence d-axis current reference; zero (default) = negative-sequence suppression.
  Real mIRefNd;
  /// Negative-sequence q-axis current reference; zero (default) = negative-sequence suppression.
  Real mIRefNq;

  const Attribute<Real>::Ptr mVcD;
  const Attribute<Real>::Ptr mVcQ;
  const Attribute<Real>::Ptr mIrcD;
  const Attribute<Real>::Ptr mIrcQ;
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;
  const Attribute<Real>::Ptr mOmegaPLL;
  const Attribute<Real>::Ptr mIrcNd;
  const Attribute<Real>::Ptr mIrcNq;

  /// Builds the affine real model (A,B,C,D,E,F) around (x,u): RHS + analytic Jacobian, E = f(x,u) - A*x - B*u.
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
                     Real kpCurrCtrl, Real kiCurrCtrl, Real iRefNd = 0.0,
                     Real iRefNq = 0.0);

  void initializeFromNodesAndTerminals(Real frequency) override;
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
