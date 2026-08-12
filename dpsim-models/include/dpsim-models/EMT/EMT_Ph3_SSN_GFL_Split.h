// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Partitioned three-phase grid-following inverter.
///
/// The electrical filter/network is kept as a fixed LTI SSN subsystem
///
///   x_n_dot = A_n x_n + B_u u + B_v v_inv
///   y       = C_n x_n + D_u u
///
/// with
///
///   x_n = [v_c,abc, i_f,abc]^T,
///   u   = v_terminal1 - v_terminal0,
///   y   = current entering the SSN component.
///
/// The GFL controller is represented separately as a locally linearized
/// affine state-space subsystem
///
///   x_c_dot = A_c x_c + B_c m + E_c
///   v_ref   = C_c x_c + D_c m + F_c
///
/// where
///
///   x_c = [theta_pll, phi_pll, p_filtered, q_filtered,
///          phi_d, phi_q, gamma_d, gamma_q]^T
///
/// and
///
///   m = [v_c,abc, i_grid,abc]^T.
///
/// A one-simulation-step delay is inserted between controller output and
/// electrical plant input. The delayed converter voltage is treated as a
/// known forcing term and therefore contributes only to the SSN history
/// current. Consequently, the Norton conductance mW remains constant for the
/// complete simulation.
class SSN_GFL_Split final : public TwoTerminalVTypeSSNComp,
                            public SharedFactory<SSN_GFL_Split> {
private:
  static constexpr Int mNetworkStateSize = 6;
  static constexpr Int mTerminalInputSize = 3;
  static constexpr Int mOutputSize = 3;

  static constexpr Int mControllerStateSize = 8;
  static constexpr Int mControllerInputSize = 6;
  static constexpr Int mControllerOutputSize = 3;

  enum NetworkStateIndex : Int {
    VcA = 0,
    VcB = 1,
    VcC = 2,
    IfA = 3,
    IfB = 4,
    IfC = 5
  };

  enum ControllerStateIndex : Int {
    ThetaPLL = 0,
    PhiPLL = 1,
    PFiltered = 2,
    QFiltered = 3,
    PhiD = 4,
    PhiQ = 5,
    GammaD = 6,
    GammaQ = 7
  };

  // -------------------------------------------------------------------------
  // Electrical parameters
  // -------------------------------------------------------------------------
  Real mLf;
  Real mCf;
  Real mRf;
  Real mRc;

  // -------------------------------------------------------------------------
  // Controller parameters
  // -------------------------------------------------------------------------
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

  // -------------------------------------------------------------------------
  // Fixed electrical plant
  //
  // The inherited matrices mA, mB, mC, mD contain the SSN-facing plant with
  // terminal voltage u as the only nodal input.
  //
  // mBConverter contains the additional known converter-voltage input:
  //
  //   x_n_dot = mA x_n + mB u + mBConverter v_inv_delayed
  // -------------------------------------------------------------------------
  Matrix mBConverter;
  Matrix mdBConverter;

  // -------------------------------------------------------------------------
  // Time-varying controller subsystem
  // -------------------------------------------------------------------------
  Matrix mControllerState;
  Matrix mControllerMeasurementOld;

  Matrix mControllerA;
  Matrix mControllerB;
  Matrix mControllerC;
  Matrix mControllerD;
  Matrix mControllerE;
  Matrix mControllerF;

  Matrix mControllerAd;
  Matrix mControllerBd;
  Matrix mControllerEd;

  // Controller output and z^-1 output.
  Matrix mConverterVoltageReference;
  Matrix mConverterVoltageDelayed;

  // -------------------------------------------------------------------------
  // Logging attributes
  // -------------------------------------------------------------------------
  const Attribute<Real>::Ptr mVcD;
  const Attribute<Real>::Ptr mVcQ;
  const Attribute<Real>::Ptr mIrcD;
  const Attribute<Real>::Ptr mIrcQ;
  const Attribute<Real>::Ptr mPInst;
  const Attribute<Real>::Ptr mQInst;
  const Attribute<Real>::Ptr mOmegaPLL;

  const Attribute<Real>::Ptr mVInvRefA;
  const Attribute<Real>::Ptr mVInvRefB;
  const Attribute<Real>::Ptr mVInvRefC;

  // -------------------------------------------------------------------------
  // Transformations
  // -------------------------------------------------------------------------
  Matrix getParkTransformMatrix(Real theta) const;
  Matrix getInverseParkTransformMatrix(Real theta) const;

  // -------------------------------------------------------------------------
  // Controller model
  // -------------------------------------------------------------------------
  void evaluateControllerStateDerivative(const Matrix &xController,
                                         const Matrix &measurement,
                                         Matrix &stateDerivative) const;

  void evaluateControllerOutput(const Matrix &xController,
                                const Matrix &measurement,
                                Matrix &output) const;

  void calculateControllerAnalyticalJacobians(const Matrix &xController,
                                              const Matrix &measurement,
                                              Matrix &A, Matrix &B, Matrix &C,
                                              Matrix &D) const;

  void buildControllerStateSpaceModel(const Matrix &xController,
                                      const Matrix &measurement, Matrix &A,
                                      Matrix &B, Matrix &C, Matrix &D,
                                      Matrix &E, Matrix &F) const;

  void recomputeControllerDiscreteModel();

  Matrix buildControllerMeasurement(const Matrix &networkState,
                                    const Matrix &terminalVoltage) const;

  // -------------------------------------------------------------------------
  // Fixed SSN plant hooks
  // -------------------------------------------------------------------------

  /// \brief Add the delayed converter-voltage forcing to the Norton history
  /// current while leaving mW unchanged.
  Matrix calculateHistoryVector() const override final;

  /// \brief Update the fixed plant state and then the separate controller.
  ///
  /// The newly calculated converter reference is stored in the z^-1 buffer and
  /// is therefore used only during the following simulation step.
  void updateState(const Matrix &uOld, const Matrix &uNew) override final;

  /// \brief Discretize the fixed plant once and form the constant Norton
  /// conductance. Also initializes the controller discretization.
  void recomputeDiscreteModel() override final;

  void updateLogAttributes(const Matrix &u) const override final;

public:
  using SharedFactory<SSN_GFL_Split>::make;

  SSN_GFL_Split(String uid, String name,
                Logger::Level logLevel = Logger::Level::off);

  SSN_GFL_Split(String name, Logger::Level logLevel = Logger::Level::off)
      : SSN_GFL_Split(name, name, logLevel) {}

  std::vector<String> getLocalStateNames() const override final;

  std::vector<SSNComp::LocalAbcStateBlock>
  getLocalAbcStateBlocks() const override final;

  /// \brief Configure the filter, PLL, power controller and current controller.
  ///
  /// The signature intentionally matches SSN_GFL / the former
  /// AvVoltSourceInverterStateSpace model.
  void setParameters(Real lf, Real cf, Real rf, Real rc, Real omegaN,
                     Real kpPLL, Real kiPLL, Real omegaCutoff, Real pRef,
                     Real qRef, Real kpPowerCtrl, Real kiPowerCtrl,
                     Real kpCurrCtrl, Real kiCurrCtrl);

  void initializeFromNodesAndTerminals(Real frequency) override final;

  // -------------------------------------------------------------------------
  // Inspection helpers
  // -------------------------------------------------------------------------

  /// \brief Combined state in the same logical order as the former monolithic
  /// GFL: 8 controller states followed by 6 electrical states.
  Matrix getState() const;

  Matrix getControllerState() const;
  Matrix getNetworkState() const;
  Matrix getStateDerivative() const;

  Matrix getInterfaceVoltage() const;
  Matrix getInterfaceCurrent() const;

  Matrix getConverterVoltageReference() const;
  Matrix getDelayedConverterVoltage() const;

  // Current local controller matrices.
  Matrix getControllerA() const;
  Matrix getControllerB() const;
  Matrix getControllerC() const;
  Matrix getControllerD() const;
  Matrix getControllerE() const;
  Matrix getControllerF() const;

  // Constant electrical-network matrices. B and D are returned for the full
  // input vector [u_abc; v_inv_abc].
  Matrix getNetworkA() const;
  Matrix getNetworkB() const;
  Matrix getNetworkC() const;
  Matrix getNetworkD() const;

  /// \brief Constant SSN Norton conductance G = mW.
  Matrix getEquivalentConductance() const;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
