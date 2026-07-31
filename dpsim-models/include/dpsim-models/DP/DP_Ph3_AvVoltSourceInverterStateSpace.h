// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <array>

#include <dpsim-models/DP/DP_Ph3_MixedVTypeVariableSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// @brief Averaged grid-following VSI (SSN), optional dual-sequence current control.
class AvVoltSourceInverterStateSpace final
    : public MixedVTypeVariableSSNComp,
      public SharedFactory<AvVoltSourceInverterStateSpace> {
private:
  /// Packed state indices: envelope block, then the contiguous control block.
  /// GammaND/GammaNQ exist only when the negative-sequence loop is enabled.
  enum StateIndex : Int {
    VcARe = 0,
    VcAIm = 1,
    VcBRe = 2,
    VcBIm = 3,
    VcCRe = 4,
    VcCIm = 5,
    IfARe = 6,
    IfAIm = 7,
    IfBRe = 8,
    IfBIm = 9,
    IfCRe = 10,
    IfCIm = 11,
    Psi = 12,
    PhiPLL = 13,
    PFiltered = 14,
    QFiltered = 15,
    PhiD = 16,
    PhiQ = 17,
    GammaD = 18,
    GammaQ = 19,
    GammaND = 20,
    GammaNQ = 21
  };

  /// One complex quantity per phase.
  using Complex3 = std::array<Complex, 3>;

  /// Enables the baseband negative-sequence current-control loop (+2 states).
  const Bool mEnableNegSeqControl;

  static constexpr std::array<Int, 3> mVcReCol = {VcARe, VcBRe, VcCRe};
  static constexpr std::array<Int, 3> mVcImCol = {VcAIm, VcBIm, VcCIm};
  static constexpr std::array<Int, 3> mIfReCol = {IfARe, IfBRe, IfCRe};
  static constexpr std::array<Int, 3> mIfImCol = {IfAIm, IfBIm, IfCIm};
  static constexpr std::array<Int, 3> mUReCol = {0, 2, 4};
  static constexpr std::array<Int, 3> mUImCol = {1, 3, 5};

  /// Positive-sequence control states the bridge-voltage reference depends on.
  static constexpr std::array<Int, 7> mOwnCol = {
      Psi, PFiltered, QFiltered, PhiD, PhiQ, GammaD, GammaQ};

  Real mLf = 0.0;
  Real mCf = 0.0;
  Real mRf = 0.0;
  Real mRc = 0.0;

  Real mOmegaN = 0.0;
  Real mKpPLL = 0.0;
  Real mKiPLL = 0.0;

  Real mOmegaCutoff = 0.0;
  Real mPRef = 0.0;
  Real mQRef = 0.0;
  Real mKpPowerCtrl = 0.0;
  Real mKiPowerCtrl = 0.0;
  Real mKpCurrCtrl = 0.0;
  Real mKiCurrCtrl = 0.0;

  /// Negative-sequence d-axis current reference; zero (default) = negative-sequence suppression.
  Real mIRefNd = 0.0;
  /// Negative-sequence q-axis current reference; zero (default) = negative-sequence suppression.
  Real mIRefNq = 0.0;

  const Attribute<Real>::Ptr mVcD;
  const Attribute<Real>::Ptr mVcQ;
  const Attribute<Real>::Ptr mIrcD;
  const Attribute<Real>::Ptr mIrcQ;
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;
  const Attribute<Real>::Ptr mOmegaPLL;
  const Attribute<Real>::Ptr mIrcNd;
  const Attribute<Real>::Ptr mIrcNq;

  /// Negative-sequence measurement, bridge-voltage reference and the conj(vc_p) coefficients of the measurement.
  struct NegSeqTerms {
    Complex ircNDQ{0.0, 0.0};
    Complex vRefNDQ{0.0, 0.0};
    Complex vRefNEnv0{0.0, 0.0};
    Complex3 hIrcN{};
  };

  /// Sensitivities of the phase-a bridge-voltage reference envelopes to states and inputs.
  struct RefSensitivities {
    /// d(vRefEnv0)/d(state) for the mOwnCol states, in that order.
    std::array<Complex, 7> posOwn{};
    Complex3 posVcRe{};
    Complex3 posVcIm{};
    Complex3 posURe{};
    Complex3 posUIm{};
    Complex negPsi{0.0, 0.0};
    Complex negGammaND{0.0, 0.0};
    Complex negGammaNQ{0.0, 0.0};
    Complex3 negVcRe{};
    Complex3 negVcIm{};
    Complex3 negURe{};
    Complex3 negUIm{};
  };

  /// Builds the affine real model (A,B,C,D,E,F) around (x,u): RHS + analytic Jacobian, E = f(x,u) - A*x - B*u.
  void buildStateSpaceModel(const Matrix &x, const Matrix &u, Matrix &A,
                            Matrix &B, Matrix &C, Matrix &D, Matrix &E,
                            Matrix &F) const;

  /// Baseband negative-sequence measurement and PI control around the operating point.
  NegSeqTerms computeNegSeqTerms(const Matrix &x, const Complex3 &vc,
                                 const Complex3 &uEnv,
                                 const Complex3 &redistFactor,
                                 const Complex3 &projCoeff,
                                 const Complex &expJPsi) const;

  /// Filter-inductor Jacobian rows: own-phase terms plus the coupling through both control chains.
  void buildInductorRows(const Complex3 &redistFactor,
                         const Complex3 &projCoeff,
                         const RefSensitivities &sens, Matrix &A,
                         Matrix &B) const;

protected:
  Bool updateComponentParameters() override final;
  void updateLogAttributes(const Matrix &u) const override final;

public:
  using SharedFactory<AvVoltSourceInverterStateSpace>::make;

  /// @param enableNegSeqControl adds the baseband negative-sequence current-control loop (+2 states); positive-sequence-only when false.
  AvVoltSourceInverterStateSpace(String uid, String name,
                                 Logger::Level logLevel = Logger::Level::off,
                                 Bool enableNegSeqControl = false);
  AvVoltSourceInverterStateSpace(String name,
                                 Logger::Level logLevel = Logger::Level::off,
                                 Bool enableNegSeqControl = false)
      : AvVoltSourceInverterStateSpace(name, name, logLevel,
                                       enableNegSeqControl) {}

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
