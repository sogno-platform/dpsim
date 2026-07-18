// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <dpsim-models/EMT/EMT_Ph3_SSN_GFM.h>

using namespace CPS;

EMT::Ph3::SSN_GFM::SSN_GFM(String uid, String name, Logger::Level logLevel)
    : TwoTerminalVTypeVariableSSNComp(uid, name, logLevel), mLf(0.0), mCf(0.0),
      mRf(0.0), mRc(0.0), mOmegaN(0.0), mNominalVoltage(0.0), mPRef(0.0),
      mQRef(0.0), mVirtualInertia(0.0), mDampingCoefficient(0.0),
      mVoltageDroopGain(0.0), mReactiveIntegralGain(0.0), mKpVoltage(0.0),
      mKiVoltage(0.0), mKpCurrent(0.0), mKiCurrent(0.0),
      mActiveDampingGain(0.0), mPowerFilterCutoff(0.0), mDelayBandwidth(0.0),
      mVirtualResistance(0.0), mVirtualReactance(0.0),
      mGridCurrentFeedforward(1.0), mReactivePowerDroop(0.0),
      mReactiveDroopCutoff(0.0), mVoltageSetpoint(0.0),
      mJacobianRelativeStep(1e-6), mJacobianAbsoluteStep(1e-8),
      mPInst(mAttributes->create<Real>("p_inst")),
      mQInst(mAttributes->create<Real>("q_inst")),
      mOmegaGFM(mAttributes->create<Real>("omega_gfm")),
      mThetaGFM(mAttributes->create<Real>("theta_gfm")),
      mVoltageMagnitudeGFM(mAttributes->create<Real>("voltage_magnitude_gfm")),
      mVcD(mAttributes->create<Real>("vc_d")),
      mVcQ(mAttributes->create<Real>("vc_q")),
      mIGridD(mAttributes->create<Real>("i_grid_d")),
      mIGridQ(mAttributes->create<Real>("i_grid_q")),
      mIfD(mAttributes->create<Real>("if_d")),
      mIfQ(mAttributes->create<Real>("if_q")),
      mVoltageReferenceD(mAttributes->create<Real>("v_ref_d")),
      mVoltageReferenceQ(mAttributes->create<Real>("v_ref_q")) {

  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  **mPInst = 0.0;
  **mQInst = 0.0;
  **mOmegaGFM = 0.0;
  **mThetaGFM = 0.0;
  **mVoltageMagnitudeGFM = 0.0;

  **mVcD = 0.0;
  **mVcQ = 0.0;

  **mIGridD = 0.0;
  **mIGridQ = 0.0;

  **mIfD = 0.0;
  **mIfQ = 0.0;

  **mVoltageReferenceD = 0.0;
  **mVoltageReferenceQ = 0.0;
}
std::vector<String> EMT::Ph3::SSN_GFM::getLocalStateNames() const {
  return {
      "p_filtered",
      "q_filtered",
      "omega",
      "theta",
      "voltage_magnitude",

      "voltage_integrator_d",
      "voltage_integrator_q",

      "current_integrator_d",
      "current_integrator_q",

      "delay_voltage_d",
      "delay_voltage_q",

      "vc_a",
      "vc_b",
      "vc_c",

      "if_a",
      "if_b",
      "if_c",
  };
}

std::vector<EMT::SSNComp::LocalAbcStateBlock>
EMT::Ph3::SSN_GFM::getLocalAbcStateBlocks() const {
  return {
      {{static_cast<Int>(VcA), static_cast<Int>(VcB), static_cast<Int>(VcC)},
       "vc"},

      {{static_cast<Int>(IfA), static_cast<Int>(IfB), static_cast<Int>(IfC)},
       "if"},
  };
}

void EMT::Ph3::SSN_GFM::setParameters(
    Real lf, Real cf, Real rf, Real rc, Real nominalVoltage, Real omegaN,
    Real pRef, Real qRef, Real virtualInertia, Real dampingCoefficient,
    Real voltageDroopGain, Real reactiveIntegralGain, Real kpVoltage,
    Real kiVoltage, Real kpCurrent, Real kiCurrent, Real activeDampingGain,
    Real powerFilterCutoff, Real delayBandwidth) {

  if (lf <= 0.0)
    throw std::invalid_argument("Filter inductance lf must be positive.");

  if (cf <= 0.0)
    throw std::invalid_argument("Filter capacitance cf must be positive.");

  if (rf < 0.0)
    throw std::invalid_argument("Filter resistance rf must be non-negative.");

  if (rc <= 0.0)
    throw std::invalid_argument("Coupling resistance rc must be positive.");

  if (nominalVoltage <= 0.0)
    throw std::invalid_argument("Nominal voltage must be positive.");

  if (omegaN <= 0.0)
    throw std::invalid_argument(
        "Nominal angular frequency omegaN must be positive.");

  if (virtualInertia <= 0.0)
    throw std::invalid_argument("Virtual inertia must be positive.");

  if (dampingCoefficient < 0.0)
    throw std::invalid_argument("Damping coefficient must be non-negative.");

  if (kiVoltage == 0.0)
    throw std::invalid_argument(
        "Voltage-controller integral gain must be non-zero.");

  if (kiCurrent == 0.0)
    throw std::invalid_argument(
        "Current-controller integral gain must be non-zero.");

  if (powerFilterCutoff < 0.0)
    throw std::invalid_argument(
        "Power-filter cutoff frequency must be non-negative.");

  if (delayBandwidth <= 0.0)
    throw std::invalid_argument("Delay bandwidth must be positive.");

  mLf = lf;
  mCf = cf;
  mRf = rf;
  mRc = rc;

  mNominalVoltage = nominalVoltage;
  mOmegaN = omegaN;

  mPRef = pRef;
  mQRef = qRef;

  mVirtualInertia = virtualInertia;
  mDampingCoefficient = dampingCoefficient;
  mVoltageDroopGain = voltageDroopGain;
  mReactiveIntegralGain = reactiveIntegralGain;

  mKpVoltage = kpVoltage;
  mKiVoltage = kiVoltage;

  mKpCurrent = kpCurrent;
  mKiCurrent = kiCurrent;

  mActiveDampingGain = activeDampingGain;
  mPowerFilterCutoff = powerFilterCutoff;
  mDelayBandwidth = delayBandwidth;

  // Only establish the dimensions here. Linearizing at x = 0, u = 0 is
  // invalid for this model because omega, voltage magnitude and the Park
  // transformation do not represent a physical operating point there.
  // The first actual local model is created after operating-point
  // initialization in initializeFromNodesAndTerminals().
  VTypeVariableSSNComp::setParameters(Matrix::Zero(mStateSize, mStateSize),
                                      Matrix::Zero(mStateSize, mInputSize),
                                      Matrix::Zero(mOutputSize, mStateSize),
                                      Matrix::Zero(mOutputSize, mInputSize),
                                      Matrix::Zero(mStateSize, 1),
                                      Matrix::Zero(mOutputSize, 1));
}

void EMT::Ph3::SSN_GFM::setNumericalLinearizationParameters(Real relativeStep,
                                                            Real absoluteStep) {

  if (relativeStep <= 0.0)
    throw std::invalid_argument(
        "Relative finite-difference step must be positive.");

  if (absoluteStep <= 0.0)
    throw std::invalid_argument(
        "Absolute finite-difference step must be positive.");

  mJacobianRelativeStep = relativeStep;
  mJacobianAbsoluteStep = absoluteStep;
}

void EMT::Ph3::SSN_GFM::setVirtualImpedance(Real virtualResistance,
                                            Real virtualReactance) {
  if (virtualResistance < 0.0)
    throw std::invalid_argument("Virtual resistance must be non-negative.");

  mVirtualResistance = virtualResistance;
  mVirtualReactance = virtualReactance;
}

void EMT::Ph3::SSN_GFM::setGridCurrentFeedforward(Real scale) {
  mGridCurrentFeedforward = scale;
}

void EMT::Ph3::SSN_GFM::setReactivePowerDroop(Real droopGain, Real cutoff) {
  if (cutoff < 0.0)
    throw std::invalid_argument("Reactive-droop cutoff must be non-negative.");

  mReactivePowerDroop = droopGain;
  mReactiveDroopCutoff = cutoff;
}

Matrix EMT::Ph3::SSN_GFM::getParkTransformMatrix(Real theta) const {

  theta = std::remainder(theta, 2.0 * PI);
  Matrix transform(2, 3);

  constexpr Real scale = 2.0 / 3.0;

  transform.row(0) << scale * std::cos(theta),
      scale * std::cos(theta - 2.0 * PI / 3.0),
      scale * std::cos(theta + 2.0 * PI / 3.0);

  transform.row(1) << -scale * std::sin(theta),
      -scale * std::sin(theta - 2.0 * PI / 3.0),
      -scale * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

Matrix EMT::Ph3::SSN_GFM::getInverseParkTransformMatrix(Real theta) const {

  theta = std::remainder(theta, 2.0 * PI);
  Matrix transform(3, 2);

  transform << std::cos(theta), -std::sin(theta),

      std::cos(theta - 2.0 * PI / 3.0), -std::sin(theta - 2.0 * PI / 3.0),

      std::cos(theta + 2.0 * PI / 3.0), -std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

Real EMT::Ph3::SSN_GFM::regularizedOmega(Real omega) const {
  constexpr Real minimumOmega = 1.0;

  if (std::abs(omega) >= minimumOmega)
    return omega;

  return omega >= 0.0 ? minimumOmega : -minimumOmega;
}

void EMT::Ph3::SSN_GFM::evaluateStateDerivative(const Matrix &x,
                                                const Matrix &u,
                                                Matrix &stateDerivative) const {

  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFM state vector has an invalid dimension.");

  if (u.rows() != mInputSize || u.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFM input vector has an invalid dimension.");

  stateDerivative.setZero(mStateSize, 1);

  const Real pFiltered = x(PFiltered, 0);
  const Real qFiltered = x(QFiltered, 0);
  const Real omega = x(Omega, 0);
  const Real theta = x(Theta, 0);
  const Real voltageMagnitude = x(VoltageMagnitude, 0);

  const Real voltageIntegratorD = x(VoltageIntegratorD, 0);
  const Real voltageIntegratorQ = x(VoltageIntegratorQ, 0);

  const Real currentIntegratorD = x(CurrentIntegratorD, 0);
  const Real currentIntegratorQ = x(CurrentIntegratorQ, 0);

  const Real delayVoltageD = x(DelayVoltageD, 0);
  const Real delayVoltageQ = x(DelayVoltageQ, 0);

  const Matrix vcAbc = x.block(VcA, 0, 3, 1);
  const Matrix ifAbc = x.block(IfA, 0, 3, 1);

  const Matrix parkTransform = getParkTransformMatrix(theta);
  const Matrix inverseParkTransform = getInverseParkTransformMatrix(theta);

  // Positive physical grid current means inverter injection into the grid.
  const Matrix iGridAbc = (vcAbc - u) / mRc;

  const Matrix vcDq = parkTransform * vcAbc;
  const Matrix ifDq = parkTransform * ifAbc;
  const Matrix iGridDq = parkTransform * iGridAbc;

  const Real vcD = vcDq(0, 0);
  const Real vcQ = vcDq(1, 0);

  const Real ifD = ifDq(0, 0);
  const Real ifQ = ifDq(1, 0);

  const Real iGridD = iGridDq(0, 0);
  const Real iGridQ = iGridDq(1, 0);

  // Capacitor current.
  const Real iCapD = ifD - iGridD;
  const Real iCapQ = ifQ - iGridQ;

  // The Park transformation is amplitude invariant (2/3 scaling).
  // Therefore three-phase instantaneous power requires the factor 3/2.
  const Real pInstantaneous = 1.5 * (vcD * iGridD + vcQ * iGridQ);

  const Real qInstantaneous = 1.5 * (vcQ * iGridD - vcD * iGridQ);

  const Real pccVoltageMagnitude = std::sqrt(vcD * vcD + vcQ * vcQ);

  // ----------------------------------------------------------------------
  // 1. Measurement filters
  // ----------------------------------------------------------------------

  stateDerivative(PFiltered, 0) =
      mPowerFilterCutoff * (pInstantaneous - pFiltered);

  stateDerivative(QFiltered, 0) =
      mPowerFilterCutoff * (qInstantaneous - qFiltered);

  // ----------------------------------------------------------------------
  // 2. VSG algorithm
  //
  // J * omega_dot =
  //     (P_ref - P_e) / omega
  //     - D * (omega - omega_n)
  //
  // theta_dot = omega
  // ----------------------------------------------------------------------

  stateDerivative(Omega, 0) = ((mPRef - pFiltered) / regularizedOmega(omega) -
                               mDampingCoefficient * (omega - mOmegaN)) /
                              mVirtualInertia;

  stateDerivative(Theta, 0) = omega;

  // ----------------------------------------------------------------------
  // 3. Reactive-power/voltage excitation controller
  //
  // E_dot =
  //     Kq * (Q_ref - Q_e)
  //     + Ku * (U_n - U_pcc)
  // ----------------------------------------------------------------------

  if (mReactiveDroopCutoff > 0.0) {
    // Proportional Q-V droop (opt-in, grid-connected): E lags a droop target
    // E_set + Dq*(Qref-Qf). No integral, so no reactive windup on a stiff grid.
    const Real droopTarget =
        mVoltageSetpoint + mReactivePowerDroop * (mQRef - qFiltered);
    stateDerivative(VoltageMagnitude, 0) =
        mReactiveDroopCutoff * (droopTarget - voltageMagnitude);
  } else {
    // Integral excitation (default, islanded).
    stateDerivative(VoltageMagnitude, 0) =
        mReactiveIntegralGain * (mQRef - qFiltered) +
        mVoltageDroopGain * (mNominalVoltage - pccVoltageMagnitude);
  }

  // EMF minus virtual-impedance drop Zv * if (Zv = Rv + jXv; zero = islanded).
  // if (a filter-current state) avoids the 1/Rc sensitivity of iGrid.
  const Real voltageReferenceD =
      voltageMagnitude - (mVirtualResistance * ifD - mVirtualReactance * ifQ);
  const Real voltageReferenceQ =
      -(mVirtualResistance * ifQ + mVirtualReactance * ifD);

  const Real voltageErrorD = voltageReferenceD - vcD;
  const Real voltageErrorQ = voltageReferenceQ - vcQ;

  stateDerivative(VoltageIntegratorD, 0) = voltageErrorD;
  stateDerivative(VoltageIntegratorQ, 0) = voltageErrorQ;

  // ----------------------------------------------------------------------
  // 4. Voltage controller
  //
  // The feed-forward and decoupling terms correspond to:
  //
  // Cf * dv_d/dt = if_d - ig_d + omega * Cf * v_q
  // Cf * dv_q/dt = if_q - ig_q - omega * Cf * v_d
  // ----------------------------------------------------------------------

  const Real currentReferenceD =
      mGridCurrentFeedforward * iGridD - omega * mCf * vcQ +
      mKpVoltage * voltageErrorD + mKiVoltage * voltageIntegratorD;

  const Real currentReferenceQ =
      mGridCurrentFeedforward * iGridQ + omega * mCf * vcD +
      mKpVoltage * voltageErrorQ + mKiVoltage * voltageIntegratorQ;

  const Real currentErrorD = currentReferenceD - ifD;
  const Real currentErrorQ = currentReferenceQ - ifQ;

  stateDerivative(CurrentIntegratorD, 0) = currentErrorD;
  stateDerivative(CurrentIntegratorQ, 0) = currentErrorQ;

  // ----------------------------------------------------------------------
  // 5. Current controller and active damping
  //
  // Lf * dif_d/dt =
  //     v_inv_d - vc_d - Rf * if_d + omega * Lf * if_q
  //
  // Lf * dif_q/dt =
  //     v_inv_q - vc_q - Rf * if_q - omega * Lf * if_d
  // ----------------------------------------------------------------------

  const Real converterVoltageReferenceD =
      vcD - omega * mLf * ifQ + mKpCurrent * currentErrorD +
      mKiCurrent * currentIntegratorD - mActiveDampingGain * iCapD;

  const Real converterVoltageReferenceQ =
      vcQ + omega * mLf * ifD + mKpCurrent * currentErrorQ +
      mKiCurrent * currentIntegratorQ - mActiveDampingGain * iCapQ;

  // ----------------------------------------------------------------------
  // 6. First-order converter/digital-delay approximation
  // ----------------------------------------------------------------------

  stateDerivative(DelayVoltageD, 0) =
      mDelayBandwidth * (converterVoltageReferenceD - delayVoltageD);

  stateDerivative(DelayVoltageQ, 0) =
      mDelayBandwidth * (converterVoltageReferenceQ - delayVoltageQ);

  Matrix converterVoltageDq(2, 1);
  converterVoltageDq << delayVoltageD, delayVoltageQ;

  // ----------------------------------------------------------------------
  // 7. Electrical filter model in abc coordinates
  //
  // vc_dot = (if + (u - vc) / Rc) / Cf
  // if_dot = (v_inv - vc - Rf * if) / Lf
  // ----------------------------------------------------------------------

  const Matrix converterVoltageAbc = inverseParkTransform * converterVoltageDq;

  const Matrix vcDerivative = (ifAbc + (u - vcAbc) / mRc) / mCf;

  const Matrix ifDerivative = (converterVoltageAbc - vcAbc - mRf * ifAbc) / mLf;

  stateDerivative.block(VcA, 0, 3, 1) = vcDerivative;
  stateDerivative.block(IfA, 0, 3, 1) = ifDerivative;
}

void EMT::Ph3::SSN_GFM::evaluateOutput(const Matrix &x, const Matrix &u,
                                       Matrix &output) const {

  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFM state vector has an invalid dimension.");

  if (u.rows() != mInputSize || u.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFM input vector has an invalid dimension.");

  const Matrix vcAbc = x.block(VcA, 0, 3, 1);

  // SSN current entering the component.
  //
  // The physical inverter injection current is:
  //
  //   i_injection = (vc - u) / Rc
  //
  // The component current used by the SSN stamp is the opposite:
  //
  //   y = -i_injection = (u - vc) / Rc
  output = (u - vcAbc) / mRc;
}

void EMT::Ph3::SSN_GFM::calculateNumericalJacobians(const Matrix &x,
                                                    const Matrix &u, Matrix &A,
                                                    Matrix &B, Matrix &C,
                                                    Matrix &D) const {

  A.setZero(mStateSize, mStateSize);
  B.setZero(mStateSize, mInputSize);
  C.setZero(mOutputSize, mStateSize);
  D.setZero(mOutputSize, mInputSize);

  Matrix fPlus = Matrix::Zero(mStateSize, 1);
  Matrix fMinus = Matrix::Zero(mStateSize, 1);

  Matrix gPlus = Matrix::Zero(mOutputSize, 1);
  Matrix gMinus = Matrix::Zero(mOutputSize, 1);

  // State Jacobians A and C.
  for (Int column = 0; column < mStateSize; ++column) {
    const Real step =
        mJacobianAbsoluteStep +
        mJacobianRelativeStep * std::max(1.0, std::abs(x(column, 0)));

    Matrix xPlus = x;
    Matrix xMinus = x;

    xPlus(column, 0) += step;
    xMinus(column, 0) -= step;

    evaluateStateDerivative(xPlus, u, fPlus);
    evaluateStateDerivative(xMinus, u, fMinus);

    evaluateOutput(xPlus, u, gPlus);
    evaluateOutput(xMinus, u, gMinus);

    A.col(column) = (fPlus - fMinus) / (2.0 * step);
    C.col(column) = (gPlus - gMinus) / (2.0 * step);
  }

  // Input Jacobians B and D.
  for (Int column = 0; column < mInputSize; ++column) {
    const Real step =
        mJacobianAbsoluteStep +
        mJacobianRelativeStep * std::max(1.0, std::abs(u(column, 0)));

    Matrix uPlus = u;
    Matrix uMinus = u;

    uPlus(column, 0) += step;
    uMinus(column, 0) -= step;

    evaluateStateDerivative(x, uPlus, fPlus);
    evaluateStateDerivative(x, uMinus, fMinus);

    evaluateOutput(x, uPlus, gPlus);
    evaluateOutput(x, uMinus, gMinus);

    B.col(column) = (fPlus - fMinus) / (2.0 * step);
    D.col(column) = (gPlus - gMinus) / (2.0 * step);
  }
}

void EMT::Ph3::SSN_GFM::buildStateSpaceModel(const Matrix &x, const Matrix &u,
                                             Matrix &A, Matrix &B, Matrix &C,
                                             Matrix &D, Matrix &E,
                                             Matrix &F) const {

  calculateNumericalJacobians(x, u, A, B, C, D);

  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);
  Matrix output = Matrix::Zero(mOutputSize, 1);

  evaluateStateDerivative(x, u, stateDerivative);
  evaluateOutput(x, u, output);

  // Local affine offsets:
  //
  // E = f(x0,u0) - A*x0 - B*u0
  // F = g(x0,u0) - C*x0 - D*u0
  E = stateDerivative - A * x - B * u;
  F = output - C * x - D * u;
}

Bool EMT::Ph3::SSN_GFM::updateComponentParameters() {
  Matrix eVector;
  Matrix fVector;

  buildStateSpaceModel(**mX, **mIntfVoltage, mA, mB, mC, mD, eVector, fVector);

  setStateOffset(eVector);
  setOutputOffset(fVector);

  // The dq/abc transformations and nonlinear GFM controls make the local
  // state-space model time varying. The SSN equivalent must therefore be
  // recomputed every simulation step.
  return true;
}

void EMT::Ph3::SSN_GFM::updateLogAttributes(const Matrix &u) const {

  const Matrix &x = **mX;

  const Real theta = x(Theta, 0);

  const Matrix parkTransform = getParkTransformMatrix(theta);

  const Matrix vcAbc = x.block(VcA, 0, 3, 1);
  const Matrix ifAbc = x.block(IfA, 0, 3, 1);

  const Matrix iGridAbc = (vcAbc - u) / mRc;

  const Matrix vcDq = parkTransform * vcAbc;
  const Matrix ifDq = parkTransform * ifAbc;
  const Matrix iGridDq = parkTransform * iGridAbc;

  const Real vcD = vcDq(0, 0);
  const Real vcQ = vcDq(1, 0);

  const Real iGridD = iGridDq(0, 0);
  const Real iGridQ = iGridDq(1, 0);

  **mVcD = vcD;
  **mVcQ = vcQ;

  **mIGridD = iGridD;
  **mIGridQ = iGridQ;

  **mIfD = ifDq(0, 0);
  **mIfQ = ifDq(1, 0);

  **mPInst = 1.5 * (vcD * iGridD + vcQ * iGridQ);
  **mQInst = 1.5 * (vcQ * iGridD - vcD * iGridQ);

  **mOmegaGFM = x(Omega, 0);
  **mThetaGFM = theta;
  **mVoltageMagnitudeGFM = x(VoltageMagnitude, 0);

  **mVoltageReferenceD = x(VoltageMagnitude, 0);
  **mVoltageReferenceQ = 0.0;
}

void EMT::Ph3::SSN_GFM::initializeFromNodesAndTerminals(Real frequency) {

  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  const Real omegaInitialization = 2.0 * PI * frequency;
  const Complex imaginaryUnit(0.0, 1.0);

  const Complex powerReference(mPRef, mQRef);

  // Terminal voltage phasors using the inherited convention:
  //
  // u = terminal1 - terminal0
  const MatrixComp uPhasor = buildInitialInputFromNodes(frequency);

  MatrixComp vcPhasor = uPhasor;
  MatrixComp iInjectionPhasor = MatrixComp::Zero(3, 1);

  // Determine capacitor-voltage and injected-current phasors through the
  // coupling resistance:
  //
  // vc = u + Rc * iInjection
  for (Int iteration = 0; iteration < mInitializationMaxIterations;
       ++iteration) {

    const Complex vcA = vcPhasor(0, 0);

    if (std::abs(vcA) < mInitializationTolerance) {
      iInjectionPhasor.setZero();
      break;
    }

    // With peak-valued phase phasors and the amplitude-invariant Park
    // transform, total three-phase power is
    //
    // S = 1.5 * V_phase_peak * conj(I_phase_peak).
    const Complex currentA = std::conj(powerReference / (1.5 * vcA));

    MatrixComp nextInjectionCurrent(3, 1);

    nextInjectionCurrent << currentA, currentA * SHIFT_TO_PHASE_B,
        currentA * SHIFT_TO_PHASE_C;

    const MatrixComp nextVcPhasor = uPhasor + mRc * nextInjectionCurrent;

    iInjectionPhasor = nextInjectionCurrent;

    if ((nextVcPhasor - vcPhasor).norm() < mInitializationTolerance) {
      vcPhasor = nextVcPhasor;
      break;
    }

    vcPhasor = nextVcPhasor;
  }

  // Capacitor-current relation:
  //
  // if = iGrid + j*omega*Cf*vc
  const MatrixComp ifPhasor =
      iInjectionPhasor + imaginaryUnit * omegaInitialization * mCf * vcPhasor;

  // Converter bridge voltage:
  //
  // vInv = vc + (Rf + j*omega*Lf)*if
  const MatrixComp converterVoltagePhasor =
      vcPhasor + (mRf + imaginaryUnit * omegaInitialization * mLf) * ifPhasor;

  const Matrix vcAbc0 = vcPhasor.real();
  const Matrix ifAbc0 = ifPhasor.real();
  const Matrix iGridAbc0 = iInjectionPhasor.real();
  const Matrix converterVoltageAbc0 = converterVoltagePhasor.real();

  const Real theta0 = std::arg(vcPhasor(0, 0));

  const Matrix parkTransform = getParkTransformMatrix(theta0);

  const Matrix vcDq0 = parkTransform * vcAbc0;
  const Matrix ifDq0 = parkTransform * ifAbc0;
  const Matrix iGridDq0 = parkTransform * iGridAbc0;
  const Matrix converterVoltageDq0 = parkTransform * converterVoltageAbc0;

  const Real vcD0 = vcDq0(0, 0);
  const Real vcQ0 = vcDq0(1, 0);

  const Real ifD0 = ifDq0(0, 0);
  const Real ifQ0 = ifDq0(1, 0);

  const Real iGridD0 = iGridDq0(0, 0);
  const Real iGridQ0 = iGridDq0(1, 0);

  const Real pInitial = 1.5 * (vcD0 * iGridD0 + vcQ0 * iGridQ0);

  const Real qInitial = 1.5 * (vcQ0 * iGridD0 - vcD0 * iGridQ0);

  const Real voltageMagnitudeInitial = std::sqrt(vcD0 * vcD0 + vcQ0 * vcQ0);

  const Real iCapD0 = ifD0 - iGridD0;
  const Real iCapQ0 = ifQ0 - iGridQ0;

  Matrix x0 = Matrix::Zero(mStateSize, 1);

  x0(PFiltered, 0) = pInitial;
  x0(QFiltered, 0) = qInitial;

  x0(Omega, 0) = omegaInitialization;
  x0(Theta, 0) = theta0;

  // EMF carries the virtual-impedance drop so vc stays at the target.
  const Real virtualDropD0 =
      mVirtualResistance * iGridD0 - mVirtualReactance * iGridQ0;
  const Real virtualReferenceQ0 =
      -(mVirtualResistance * iGridQ0 + mVirtualReactance * iGridD0);
  x0(VoltageMagnitude, 0) = voltageMagnitudeInitial + virtualDropD0;

  // Proportional-droop setpoint: the operating EMF, so the droop is centered at
  // the initial point (E_dot = 0 when Qf = Qref at t = 0).
  mVoltageSetpoint = x0(VoltageMagnitude, 0);

  // Voltage controller:
  //
  // iRefD =
  //     iGridD - omega*Cf*vcQ
  //     + KpV*(E-vcD)
  //     + KiV*xiVd
  //
  // Set iRefD = ifD and solve for xiVd.
  x0(VoltageIntegratorD, 0) = (ifD0 - mGridCurrentFeedforward * iGridD0 +
                               omegaInitialization * mCf * vcQ0 -
                               mKpVoltage * (voltageMagnitudeInitial - vcD0)) /
                              mKiVoltage;

  // iRefQ =
  //     iGridQ + omega*Cf*vcD
  //     + KpV*(0-vcQ)
  //     + KiV*xiVq
  //
  // Set iRefQ = ifQ and solve for xiVq.
  x0(VoltageIntegratorQ, 0) = (ifQ0 - mGridCurrentFeedforward * iGridQ0 -
                               omegaInitialization * mCf * vcD0 -
                               mKpVoltage * (virtualReferenceQ0 - vcQ0)) /
                              mKiVoltage;

  // Current controller steady-state integrators.
  //
  // vInvD =
  //     vcD - omega*Lf*ifQ
  //     + KpI*(iRefD-ifD)
  //     + KiI*xiId
  //     - Kc*iCapD
  x0(CurrentIntegratorD, 0) =
      (converterVoltageDq0(0, 0) - vcD0 + omegaInitialization * mLf * ifQ0 +
       mActiveDampingGain * iCapD0) /
      mKiCurrent;

  // vInvQ =
  //     vcQ + omega*Lf*ifD
  //     + KpI*(iRefQ-ifQ)
  //     + KiI*xiIq
  //     - Kc*iCapQ
  x0(CurrentIntegratorQ, 0) =
      (converterVoltageDq0(1, 0) - vcQ0 - omegaInitialization * mLf * ifD0 +
       mActiveDampingGain * iCapQ0) /
      mKiCurrent;

  // At steady state, the first-order delay output equals its input.
  x0(DelayVoltageD, 0) = converterVoltageDq0(0, 0);
  x0(DelayVoltageQ, 0) = converterVoltageDq0(1, 0);

  x0.block(VcA, 0, 3, 1) = vcAbc0;
  x0.block(IfA, 0, 3, 1) = ifAbc0;

  // Store the initialized state and terminal voltage.
  **mX = x0;
  **mIntfVoltage = uPhasor.real();

  // Do not call updateComponentParameters() directly here.
  //
  // updateStateSpaceModel() performs:
  //   1. updateComponentParameters()
  //   2. recomputeDiscreteModel()
  //
  // This initializes mdA, mdB, mdE and mW.
  updateStateSpaceModel();

  // The MNA right-side-vector stamp uses mYHist. It must therefore already
  // be valid before the first network solution.
  mYHist = calculateHistoryVector();

  // Use the same Norton-equivalent equation as the SSN network stamp:
  //
  //   y = W*u + yHist
  //
  // This avoids an initialization mismatch between mIntfCurrent and the
  // actual component stamp.

  **mIntfCurrent = mW * (**mIntfVoltage) + mYHist;

  updateLogAttributes(**mIntfVoltage);

  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);

  evaluateStateDerivative(**mX, **mIntfVoltage, stateDerivative);

  const Matrix nonlinearOutput = [&]() {
    Matrix output = Matrix::Zero(mOutputSize, 1);
    evaluateOutput(**mX, **mIntfVoltage, output);
    return output;
  }();

  const Matrix ssnOutput = mW * (**mIntfVoltage) + mYHist;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- SSN GFM initialization ---"
      "\nInput voltage u: {:s}"
      "\nInterface current y: {:s}"
      "\nState x: {:s}"
      "\nState derivative norm: {:.6e}"
      "\nNonlinear output: {:s}"
      "\nSSN output: {:s}"
      "\nOutput mismatch norm: {:.6e}"
      "\nW norm: {:.6e}"
      "\nHistory-vector norm: {:.6e}"
      "\nP/Q initial: [{:.6e}, {:.6e}]"
      "\nVc dq: [{:.6e}, {:.6e}]"
      "\nIGrid dq: [{:.6e}, {:.6e}]"
      "\nIf dq: [{:.6e}, {:.6e}]"
      "\nConverter voltage dq: [{:.6e}, {:.6e}]"
      "\n--- SSN GFM initialization finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent), Logger::matrixToString(**mX),
      stateDerivative.norm(), Logger::matrixToString(nonlinearOutput),
      Logger::matrixToString(ssnOutput), (nonlinearOutput - ssnOutput).norm(),
      mW.norm(), mYHist.norm(), pInitial, qInitial, vcD0, vcQ0, iGridD0,
      iGridQ0, ifD0, ifQ0, converterVoltageDq0(0, 0),
      converterVoltageDq0(1, 0));
}

Matrix EMT::Ph3::SSN_GFM::getState() const { return **mX; }

Matrix EMT::Ph3::SSN_GFM::getStateDerivative() const {
  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);

  evaluateStateDerivative(**mX, **mIntfVoltage, stateDerivative);

  return stateDerivative;
}

Matrix EMT::Ph3::SSN_GFM::getInterfaceVoltage() const { return **mIntfVoltage; }

Matrix EMT::Ph3::SSN_GFM::getInterfaceCurrent() const { return **mIntfCurrent; }
