// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <array>

#include <dpsim-models/DP/DP_Ph3_TwoTerminalVTypeSplitSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// Positive-sequence grid-following DP inverter with a fixed electrical plant
/// and a separately linearized controller connected through a one-step delay.
class SSN_GFL_Split final : public TwoTerminalVTypeSplitSSNComp,
                            public SharedFactory<SSN_GFL_Split> {
private:
  static constexpr Int mNetworkStateSize = 12;
  static constexpr Int mTerminalInputSize = 6;
  static constexpr Int mOutputSize = 6;
  static constexpr Int mControllerStateSize = 8;
  static constexpr Int mControllerInputSize = 12;
  static constexpr Int mControllerOutputSize = 6;

  enum ControllerStateIndex : Int {
    Psi = 0,
    PhiPLL = 1,
    PFiltered = 2,
    QFiltered = 3,
    PhiD = 4,
    PhiQ = 5,
    GammaD = 6,
    GammaQ = 7
  };

  static constexpr std::array<Int, 3> mVcRe = {0, 2, 4};
  static constexpr std::array<Int, 3> mVcIm = {1, 3, 5};
  static constexpr std::array<Int, 3> mIfRe = {6, 8, 10};
  static constexpr std::array<Int, 3> mIfIm = {7, 9, 11};
  static constexpr std::array<Int, 3> mMeasVcRe = {0, 2, 4};
  static constexpr std::array<Int, 3> mMeasVcIm = {1, 3, 5};
  static constexpr std::array<Int, 3> mMeasIRe = {6, 8, 10};
  static constexpr std::array<Int, 3> mMeasIIm = {7, 9, 11};

  using Complex3 = std::array<Complex, 3>;

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

  const Attribute<Real>::Ptr mVcD;
  const Attribute<Real>::Ptr mVcQ;
  const Attribute<Real>::Ptr mIrcD;
  const Attribute<Real>::Ptr mIrcQ;
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;
  const Attribute<Real>::Ptr mOmegaPLL;

  void unpackMeasurement(const Matrix &measurement, Complex3 &vc,
                         Complex3 &iGrid) const;
  void evaluateControllerStateDerivative(const Matrix &controllerState,
                                         const Matrix &measurement,
                                         Matrix &stateDerivative) const;
  void evaluateControllerOutput(const Matrix &controllerState,
                                const Matrix &measurement,
                                Matrix &output) const override final;
  void calculateControllerAnalyticalJacobians(const Matrix &controllerState,
                                              const Matrix &measurement,
                                              Matrix &A, Matrix &B, Matrix &C,
                                              Matrix &D) const;
  void buildControllerStateSpaceModel(const Matrix &controllerState,
                                      const Matrix &measurement, Matrix &A,
                                      Matrix &B, Matrix &C, Matrix &D,
                                      Matrix &E,
                                      Matrix &F) const override final;
  void updateLogAttributes(const Matrix &terminalVoltage) const override final;

public:
  using SharedFactory<SSN_GFL_Split>::make;

  SSN_GFL_Split(String uid, String name,
                Logger::Level logLevel = Logger::Level::off);
  SSN_GFL_Split(String name, Logger::Level logLevel = Logger::Level::off)
      : SSN_GFL_Split(name, name, logLevel) {}

  void setParameters(Real lf, Real cf, Real rf, Real rc, Real omegaN,
                     Real kpPLL, Real kiPLL, Real omegaCutoff, Real pRef,
                     Real qRef, Real kpPowerCtrl, Real kiPowerCtrl,
                     Real kpCurrCtrl, Real kiCurrCtrl);
  void initializeFromNodesAndTerminals(Real frequency) override final;

  std::vector<String> getSplitLocalStateNames() const override final;

  Matrix getControllerState() const;
  Matrix getNetworkState() const;
  Matrix getStateDerivative() const;
  Matrix getInterfaceVoltagePacked() const;
  Matrix getInterfaceCurrentPacked() const;
  Matrix getConverterVoltageReference() const;
  Matrix getDelayedConverterVoltage() const;
  Matrix getControllerA() const;
  Matrix getControllerB() const;
  Matrix getControllerC() const;
  Matrix getControllerD() const;
  Matrix getNetworkA() const;
  Matrix getNetworkB() const;
  Matrix getNetworkC() const;
  Matrix getNetworkD() const;
  Matrix getEquivalentConductance() const;
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
