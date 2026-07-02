// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <stdexcept>

#include <dpsim-models/EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.h>

using namespace CPS;

EMT::Ph3::AvVoltSourceInverterStateSpace::AvVoltSourceInverterStateSpace(
    String uid, String name, Logger::Level logLevel)
    : TwoTerminalVTypeVariableSSNComp(uid, name, logLevel), mLf(0.0), mCf(0.0),
      mRf(0.0), mRc(0.0), mOmegaN(0.0), mKpPLL(0.0), mKiPLL(0.0),
      mOmegaCutoff(0.0), mPRef(0.0), mQRef(0.0), mKpPowerCtrl(0.0),
      mKiPowerCtrl(0.0), mKpCurrCtrl(0.0), mKiCurrCtrl(0.0),
      mVcD(mAttributes->create<Real>("vc_d")),
      mVcQ(mAttributes->create<Real>("vc_q")),
      mIrcD(mAttributes->create<Real>("irc_d")),
      mIrcQ(mAttributes->create<Real>("irc_q")),
      mPInst(mAttributes->create<Real>("p_inst")),
      mQInst(mAttributes->create<Real>("q_inst")),
      mOmegaPLL(mAttributes->create<Real>("omega_pll")) {
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  **mVcD = 0.0;
  **mVcQ = 0.0;
  **mIrcD = 0.0;
  **mIrcQ = 0.0;
  **mPInst = 0.0;
  **mQInst = 0.0;
  **mOmegaPLL = 0.0;
}

std::vector<String>
EMT::Ph3::AvVoltSourceInverterStateSpace::getLocalStateNames() const {
  return {
      "theta_pll", "phi_pll", "p_filtered", "q_filtered", "phi_d",
      "phi_q",     "gamma_d", "gamma_q",    "vc_a",       "vc_b",
      "vc_c",      "if_a",    "if_b",       "if_c",
  };
}

std::vector<EMT::SSNComp::LocalAbcStateBlock>
EMT::Ph3::AvVoltSourceInverterStateSpace::getLocalAbcStateBlocks() const {
  return {
      {{static_cast<UInt>(VcA), static_cast<UInt>(VcB), static_cast<UInt>(VcC)},
       "vc"},
      {{static_cast<UInt>(IfA), static_cast<UInt>(IfB), static_cast<UInt>(IfC)},
       "if"},
  };
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::setParameters(
    Real lf, Real cf, Real rf, Real rc, Real omegaN, Real kpPLL, Real kiPLL,
    Real omegaCutoff, Real pRef, Real qRef, Real kpPowerCtrl, Real kiPowerCtrl,
    Real kpCurrCtrl, Real kiCurrCtrl) {
  if (lf <= 0.0)
    throw std::invalid_argument("Filter inductance lf must be positive.");

  if (cf <= 0.0)
    throw std::invalid_argument("Filter capacitance cf must be positive.");

  if (rf < 0.0)
    throw std::invalid_argument("Filter resistance rf must be non-negative.");

  if (rc <= 0.0)
    throw std::invalid_argument("Coupling resistance rc must be positive.");

  if (omegaN <= 0.0)
    throw std::invalid_argument(
        "Nominal angular frequency omegaN must be positive.");

  if (omegaCutoff < 0.0)
    throw std::invalid_argument(
        "Power-filter cutoff frequency omegaCutoff must be non-negative.");

  if (kiPLL == 0.0)
    throw std::invalid_argument("PLL integral gain kiPLL must be non-zero.");

  if (kiPowerCtrl == 0.0)
    throw std::invalid_argument(
        "Power-control integral gain kiPowerCtrl must be non-zero.");

  if (kiCurrCtrl == 0.0)
    throw std::invalid_argument(
        "Current-control integral gain kiCurrCtrl must be non-zero.");

  mLf = lf;
  mCf = cf;
  mRf = rf;
  mRc = rc;

  mOmegaN = omegaN;
  mKpPLL = kpPLL;
  mKiPLL = kiPLL;

  mOmegaCutoff = omegaCutoff;
  mPRef = pRef;
  mQRef = qRef;
  mKpPowerCtrl = kpPowerCtrl;
  mKiPowerCtrl = kiPowerCtrl;
  mKpCurrCtrl = kpCurrCtrl;
  mKiCurrCtrl = kiCurrCtrl;

  const Matrix x0 = Matrix::Zero(mStateSize, 1);
  const Matrix u0 = Matrix::Zero(3, 1);

  Matrix aMatrix;
  Matrix bMatrix;
  Matrix cMatrix;
  Matrix dMatrix;
  Matrix eVector;
  Matrix fVector;
  buildStateSpaceModel(x0, u0, aMatrix, bMatrix, cMatrix, dMatrix, eVector,
                       fVector);

  VTypeVariableSSNComp::setParameters(aMatrix, bMatrix, cMatrix, dMatrix,
                                      eVector, fVector);
}

Matrix EMT::Ph3::AvVoltSourceInverterStateSpace::getParkTransformMatrix(
    Real theta) const {
  Matrix transform(2, 3);
  const Real k = std::sqrt(2.0 / 3.0);

  transform.row(0) << k * std::cos(theta), k * std::cos(theta - 2.0 * PI / 3.0),
      k * std::cos(theta + 2.0 * PI / 3.0);

  transform.row(1) << -k * std::sin(theta),
      -k * std::sin(theta - 2.0 * PI / 3.0),
      -k * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

Matrix EMT::Ph3::AvVoltSourceInverterStateSpace::getInverseParkTransformMatrix(
    Real theta) const {
  Matrix transform(3, 2);
  const Real k = std::sqrt(2.0 / 3.0);

  transform << k * std::cos(theta), -k * std::sin(theta),
      k * std::cos(theta - 2.0 * PI / 3.0),
      -k * std::sin(theta - 2.0 * PI / 3.0),
      k * std::cos(theta + 2.0 * PI / 3.0),
      -k * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::updateLogAttributes(
    const Matrix &u) const {
  const Matrix &x = **mX;

  const Matrix parkTransform = getParkTransformMatrix(x(ThetaPLL, 0));
  const Matrix vcAbc = x.block(VcA, 0, 3, 1);
  const Matrix iGridAbc = (vcAbc - u) / mRc;

  **mVcD = (parkTransform.row(0) * vcAbc)(0, 0);
  **mVcQ = (parkTransform.row(1) * vcAbc)(0, 0);
  **mIrcD = (parkTransform.row(0) * iGridAbc)(0, 0);
  **mIrcQ = (parkTransform.row(1) * iGridAbc)(0, 0);

  **mPInst = **mVcD * **mIrcD + **mVcQ * **mIrcQ;
  **mQInst = -**mVcD * **mIrcQ + **mVcQ * **mIrcD;

  **mOmegaPLL = mOmegaN + mKpPLL * **mVcQ + mKiPLL * x(PhiPLL, 0);
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::initializeFromNodesAndTerminals(
    Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  // The generic SSN phasor initialization is not used because this component
  // mixes EMT abc electrical states with dq-frame controller states. The filter
  // states are initialized from balanced phasors; the controller states are
  // initialized algebraically from the corresponding dq operating point.

  const Real omega = 2.0 * PI * frequency;
  const Complex j(0.0, 1.0);
  const Complex powerRef(mPRef, mQRef);

  const MatrixComp uPhasor = buildInitialInputFromNodes(frequency);

  MatrixComp vcPhasor = uPhasor;
  MatrixComp iInjPhasor = MatrixComp::Zero(3, 1);

  for (Int iter = 0; iter < mInitializationMaxIterations; ++iter) {
    const Complex vcA = vcPhasor(0, 0);

    if (std::abs(vcA) < mInitializationTolerance) {
      iInjPhasor.setZero();
      break;
    }

    const Complex iA = std::conj(powerRef / (1.5 * vcA));

    MatrixComp iNext(3, 1);
    iNext << iA, iA * SHIFT_TO_PHASE_B, iA * SHIFT_TO_PHASE_C;

    const MatrixComp vcNext = uPhasor + mRc * iNext;

    iInjPhasor = iNext;

    if ((vcNext - vcPhasor).norm() < mInitializationTolerance) {
      vcPhasor = vcNext;
      break;
    }

    vcPhasor = vcNext;
  }

  const MatrixComp ifPhasor = j * omega * mCf * vcPhasor + iInjPhasor;
  const MatrixComp vRefPhasor = vcPhasor + (mRf + j * omega * mLf) * ifPhasor;

  const Matrix vcAbc0 = vcPhasor.real();
  const Matrix ifAbc0 = ifPhasor.real();
  const Matrix iInjAbc0 = iInjPhasor.real();
  const Matrix vRefAbc0 = vRefPhasor.real();

  const Real theta0 = std::arg(vcPhasor(0, 0));
  const Matrix parkTransform = getParkTransformMatrix(theta0);

  const Matrix vcDq = parkTransform * vcAbc0;
  const Matrix iDq = parkTransform * iInjAbc0;
  const Matrix vRefDq = parkTransform * vRefAbc0;

  const Real vcD = vcDq(0, 0);
  const Real vcQ = vcDq(1, 0);
  const Real ircD = iDq(0, 0);
  const Real ircQ = iDq(1, 0);

  const Real pInit = vcD * ircD + vcQ * ircQ;
  const Real qInit = -vcD * ircQ + vcQ * ircD;

  Matrix x0 = Matrix::Zero(mStateSize, 1);

  x0(ThetaPLL, 0) = theta0;
  x0(PhiPLL, 0) = (omega - mOmegaN) / mKiPLL;
  x0(PFiltered, 0) = pInit;
  x0(QFiltered, 0) = qInit;

  x0(PhiD, 0) = (ircD + mKpPowerCtrl * (pInit - mPRef)) / mKiPowerCtrl;
  x0(PhiQ, 0) = (ircQ - mKpPowerCtrl * (qInit - mQRef)) / mKiPowerCtrl;

  const Real iRefD =
      -mKpPowerCtrl * pInit + mKiPowerCtrl * x0(PhiD, 0) + mKpPowerCtrl * mPRef;
  const Real iRefQ =
      mKpPowerCtrl * qInit + mKiPowerCtrl * x0(PhiQ, 0) - mKpPowerCtrl * mQRef;

  x0(GammaD, 0) = (vRefDq(0, 0) + mKpCurrCtrl * (ircD - iRefD)) / mKiCurrCtrl;
  x0(GammaQ, 0) = (vRefDq(1, 0) + mKpCurrCtrl * (ircQ - iRefQ)) / mKiCurrCtrl;

  x0.block(VcA, 0, 3, 1) = vcAbc0;
  x0.block(IfA, 0, 3, 1) = ifAbc0;

  **mX = x0;
  **mIntfVoltage = uPhasor.real();
  **mIntfCurrent = ((uPhasor - vcPhasor) / mRc).real();

  updateComponentParameters();
  updateLogAttributes(**mIntfVoltage);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Inverter SSN phasor/dq initialization ---"
                     "\nInput u: {:s}"
                     "\nOutput y: {:s}"
                     "\nState x: {:s}"
                     "\nP/Q init: [{:.6e}, {:.6e}]"
                     "\nVc dq: [{:.6e}, {:.6e}]"
                     "\nIinj dq: [{:.6e}, {:.6e}]"
                     "\n--- Initialization finished ---",
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent),
                     Logger::matrixToString(**mX), pInit, qInit, vcD, vcQ, ircD,
                     ircQ);
}

void EMT::Ph3::AvVoltSourceInverterStateSpace::buildStateSpaceModel(
    const Matrix &x, const Matrix &u, Matrix &A, Matrix &B, Matrix &C,
    Matrix &D, Matrix &E, Matrix &F) const {
  // -------------------------------------------------------------------------
  // 1) Operating point and Park transformation
  // -------------------------------------------------------------------------
  const Real theta0 = x(ThetaPLL, 0);
  const Real pFiltered0 = x(PFiltered, 0);
  const Real qFiltered0 = x(QFiltered, 0);
  const Real phiD0 = x(PhiD, 0);
  const Real phiQ0 = x(PhiQ, 0);
  const Real gammaD0 = x(GammaD, 0);
  const Real gammaQ0 = x(GammaQ, 0);
  const Matrix vcAbc0 = x.block(VcA, 0, 3, 1);

  const Matrix identity3 = Matrix::Identity(3, 3);

  const Matrix parkTransform = getParkTransformMatrix(theta0);
  const Matrix tD = parkTransform.row(0);
  const Matrix tQ = parkTransform.row(1);
  const Matrix inverseParkTransform = getInverseParkTransformMatrix(theta0);
  const Matrix sD = inverseParkTransform.col(0);
  const Matrix sQ = inverseParkTransform.col(1);

  const Matrix dTdTheta = tQ;
  const Matrix dTqTheta = -tD;
  const Matrix dSdTheta = sQ;
  const Matrix dSqTheta = -sD;

  // -------------------------------------------------------------------------
  // 2) PLL measurement equation
  //
  // vq(theta, Vc) ≈ aThetaPLL * theta + aVPLL * Vc + bVq
  //
  // theta_dot = omegaN + KpPLL * vq + KiPLL * phiPLL
  // phiPLL_dot = vq
  // -------------------------------------------------------------------------
  const Real vq0 = (tQ * vcAbc0)(0, 0);
  const Real aThetaPLL = (dTqTheta * vcAbc0)(0, 0);
  const Matrix aVPLL = tQ;
  const Real bVq = vq0 - aThetaPLL * theta0 - (aVPLL * vcAbc0)(0, 0);

  // -------------------------------------------------------------------------
  // 3) Grid-current measurement in dq
  //
  // i_rc = (Vc - u) / Rc
  // i_dq = T(theta) * i_rc
  //
  // The controller uses positive current as inverter injection into the grid.
  // The SSN output uses the opposite sign:
  //
  // y = (u - Vc) / Rc
  // -------------------------------------------------------------------------
  const Matrix iInjAbc0 = (vcAbc0 - u) / mRc;

  const Real vcD0 = (tD * vcAbc0)(0, 0);
  const Real vcQ0 = (tQ * vcAbc0)(0, 0);
  const Real ircD0 = (tD * iInjAbc0)(0, 0);
  const Real ircQ0 = (tQ * iInjAbc0)(0, 0);

  const Matrix dVcDByVc = tD;
  const Matrix dVcQByVc = tQ;
  const Matrix dIrcDByVc = tD / mRc;
  const Matrix dIrcQByVc = tQ / mRc;
  const Matrix dIrcDByU = -tD / mRc;
  const Matrix dIrcQByU = -tQ / mRc;

  const Real dVcDByTheta = (dTdTheta * vcAbc0)(0, 0);
  const Real dVcQByTheta = (dTqTheta * vcAbc0)(0, 0);
  const Real dIrcDByTheta = (dTdTheta * iInjAbc0)(0, 0);
  const Real dIrcQByTheta = (dTqTheta * iInjAbc0)(0, 0);

  const Real bIrcD = ircD0 - dIrcDByTheta * theta0 -
                     (dIrcDByVc * vcAbc0)(0, 0) - (dIrcDByU * u)(0, 0);

  const Real bIrcQ = ircQ0 - dIrcQByTheta * theta0 -
                     (dIrcQByVc * vcAbc0)(0, 0) - (dIrcQByU * u)(0, 0);

  // -------------------------------------------------------------------------
  // 4) Power measurement and power-filter states
  //
  // p = vc_d * irc_d + vc_q * irc_q
  // q = -vc_d * irc_q + vc_q * irc_d
  //
  // P_dot = omegaCutoff * (p - P)
  // Q_dot = omegaCutoff * (q - Q)
  // -------------------------------------------------------------------------
  const Real p0 = vcD0 * ircD0 + vcQ0 * ircQ0;
  const Real dPByTheta = ircD0 * dVcDByTheta + vcD0 * dIrcDByTheta +
                         ircQ0 * dVcQByTheta + vcQ0 * dIrcQByTheta;
  const Matrix dPByVc =
      ircD0 * dVcDByVc + vcD0 * dIrcDByVc + ircQ0 * dVcQByVc + vcQ0 * dIrcQByVc;
  const Matrix dPByU = vcD0 * dIrcDByU + vcQ0 * dIrcQByU;
  const Real bP =
      p0 - dPByTheta * theta0 - (dPByVc * vcAbc0)(0, 0) - (dPByU * u)(0, 0);

  const Real q0 = -vcD0 * ircQ0 + vcQ0 * ircD0;
  const Real dQByTheta = -ircQ0 * dVcDByTheta - vcD0 * dIrcQByTheta +
                         ircD0 * dVcQByTheta + vcQ0 * dIrcDByTheta;
  const Matrix dQByVc = -ircQ0 * dVcDByVc - vcD0 * dIrcQByVc +
                        ircD0 * dVcQByVc + vcQ0 * dIrcDByVc;
  const Matrix dQByU = -vcD0 * dIrcQByU + vcQ0 * dIrcDByU;
  const Real bQ =
      q0 - dQByTheta * theta0 - (dQByVc * vcAbc0)(0, 0) - (dQByU * u)(0, 0);

  // -------------------------------------------------------------------------
  // 5) Outer power control
  //
  // phi_d_dot = Pref - P
  // phi_q_dot = Q - Qref
  //
  // iRef_d = KpP * (Pref - P) + KiP * phi_d
  // iRef_q = KpP * (Q - Qref) + KiP * phi_q
  // -------------------------------------------------------------------------
  const Real iRefD0 =
      -mKpPowerCtrl * pFiltered0 + mKiPowerCtrl * phiD0 + mKpPowerCtrl * mPRef;
  const Real iRefQ0 =
      mKpPowerCtrl * qFiltered0 + mKiPowerCtrl * phiQ0 - mKpPowerCtrl * mQRef;

  // -------------------------------------------------------------------------
  // 6) Inner current control and bridge-voltage reference
  //
  // gamma_d_dot = iRef_d - irc_d
  // gamma_q_dot = iRef_q - irc_q
  //
  // vRef_d = KpI * (iRef_d - irc_d) + KiI * gamma_d
  // vRef_q = KpI * (iRef_q - irc_q) + KiI * gamma_q
  //
  // vRef_abc = T_inv(theta) * [vRef_d, vRef_q]
  // -------------------------------------------------------------------------
  const Real vRefD0 =
      -mKpCurrCtrl * ircD0 + mKiCurrCtrl * gammaD0 + mKpCurrCtrl * iRefD0;
  const Real vRefQ0 =
      -mKpCurrCtrl * ircQ0 + mKiCurrCtrl * gammaQ0 + mKpCurrCtrl * iRefQ0;

  const Matrix vRefAbc0 = sD * vRefD0 + sQ * vRefQ0;

  const Real dVRefDByTheta = -mKpCurrCtrl * dIrcDByTheta;
  const Matrix dVRefDByVc = -mKpCurrCtrl * dIrcDByVc;
  const Matrix dVRefDByU = -mKpCurrCtrl * dIrcDByU;

  const Real dVRefQByTheta = -mKpCurrCtrl * dIrcQByTheta;
  const Matrix dVRefQByVc = -mKpCurrCtrl * dIrcQByVc;
  const Matrix dVRefQByU = -mKpCurrCtrl * dIrcQByU;

  Matrix dVRefAbcByX = Matrix::Zero(3, mStateSize);
  Matrix dVRefAbcByU = Matrix::Zero(3, 3);

  dVRefAbcByX.col(ThetaPLL) = dSdTheta * vRefD0 + dSqTheta * vRefQ0 +
                              sD * dVRefDByTheta + sQ * dVRefQByTheta;

  dVRefAbcByX.col(PFiltered) += sD * (-mKpCurrCtrl * mKpPowerCtrl);
  dVRefAbcByX.col(PhiD) += sD * (mKpCurrCtrl * mKiPowerCtrl);
  dVRefAbcByX.col(GammaD) += sD * mKiCurrCtrl;

  dVRefAbcByX.col(QFiltered) += sQ * (mKpCurrCtrl * mKpPowerCtrl);
  dVRefAbcByX.col(PhiQ) += sQ * (mKpCurrCtrl * mKiPowerCtrl);
  dVRefAbcByX.col(GammaQ) += sQ * mKiCurrCtrl;

  dVRefAbcByX.block(0, VcA, 3, 3) += sD * dVRefDByVc + sQ * dVRefQByVc;

  dVRefAbcByU = sD * dVRefDByU + sQ * dVRefQByU;

  const Matrix vRefAbcOffset = vRefAbc0 - dVRefAbcByX * x - dVRefAbcByU * u;

  // -------------------------------------------------------------------------
  // 7) Initialize affine state-space matrices
  //
  // x_dot ≈ A * x + B * u + E
  // y     ≈ C * x + D * u + F
  // -------------------------------------------------------------------------
  A.setZero(mStateSize, mStateSize);
  B.setZero(mStateSize, 3);
  C.setZero(3, mStateSize);
  D.setZero(3, 3);
  E.setZero(mStateSize, 1);
  F.setZero(3, 1);

  // -------------------------------------------------------------------------
  // 8) Stamp PLL rows
  // -------------------------------------------------------------------------
  A(ThetaPLL, ThetaPLL) = mKpPLL * aThetaPLL;
  A(ThetaPLL, PhiPLL) = mKiPLL;
  A.block(ThetaPLL, VcA, 1, 3) = mKpPLL * aVPLL;

  A(PhiPLL, ThetaPLL) = aThetaPLL;
  A.block(PhiPLL, VcA, 1, 3) = aVPLL;

  E(ThetaPLL, 0) = mOmegaN + mKpPLL * bVq;
  E(PhiPLL, 0) = bVq;

  // -------------------------------------------------------------------------
  // 9) Stamp power-filter rows
  // -------------------------------------------------------------------------
  A(PFiltered, ThetaPLL) = mOmegaCutoff * dPByTheta;
  A(PFiltered, PFiltered) = -mOmegaCutoff;
  A.block(PFiltered, VcA, 1, 3) = mOmegaCutoff * dPByVc;
  B.block(PFiltered, 0, 1, 3) = mOmegaCutoff * dPByU;
  E(PFiltered, 0) = mOmegaCutoff * bP;

  A(QFiltered, ThetaPLL) = mOmegaCutoff * dQByTheta;
  A(QFiltered, QFiltered) = -mOmegaCutoff;
  A.block(QFiltered, VcA, 1, 3) = mOmegaCutoff * dQByVc;
  B.block(QFiltered, 0, 1, 3) = mOmegaCutoff * dQByU;
  E(QFiltered, 0) = mOmegaCutoff * bQ;

  // -------------------------------------------------------------------------
  // 10) Stamp outer-loop integrator rows
  // -------------------------------------------------------------------------
  A(PhiD, PFiltered) = -1.0;
  E(PhiD, 0) = mPRef;

  A(PhiQ, QFiltered) = 1.0;
  E(PhiQ, 0) = -mQRef;

  // -------------------------------------------------------------------------
  // 11) Stamp current-loop integrator rows
  // -------------------------------------------------------------------------
  A(GammaD, PFiltered) = -mKpPowerCtrl;
  A(GammaD, PhiD) = mKiPowerCtrl;
  A(GammaD, ThetaPLL) = -dIrcDByTheta;
  A.block(GammaD, VcA, 1, 3) = -dIrcDByVc;
  B.block(GammaD, 0, 1, 3) = -dIrcDByU;
  E(GammaD, 0) = mKpPowerCtrl * mPRef - bIrcD;

  A(GammaQ, QFiltered) = mKpPowerCtrl;
  A(GammaQ, PhiQ) = mKiPowerCtrl;
  A(GammaQ, ThetaPLL) = -dIrcQByTheta;
  A.block(GammaQ, VcA, 1, 3) = -dIrcQByVc;
  B.block(GammaQ, 0, 1, 3) = -dIrcQByU;
  E(GammaQ, 0) = -mKpPowerCtrl * mQRef - bIrcQ;

  // -------------------------------------------------------------------------
  // 12) Stamp electrical filter plant
  //
  // Vc_dot = If / Cf + (u - Vc) / (Cf * Rc)
  // If_dot = (vRef_abc - Vc - Rf * If) / Lf
  // -------------------------------------------------------------------------
  A.block(VcA, VcA, 3, 3) = -1.0 / (mCf * mRc) * identity3;
  A.block(VcA, IfA, 3, 3) = 1.0 / mCf * identity3;
  B.block(VcA, 0, 3, 3) = 1.0 / (mCf * mRc) * identity3;

  A.block(IfA, 0, 3, mStateSize) = (1.0 / mLf) * dVRefAbcByX;
  A.block(IfA, VcA, 3, 3) += -1.0 / mLf * identity3;
  A.block(IfA, IfA, 3, 3) += -mRf / mLf * identity3;
  B.block(IfA, 0, 3, 3) = (1.0 / mLf) * dVRefAbcByU;
  E.block(IfA, 0, 3, 1) = (1.0 / mLf) * vRefAbcOffset;

  // -------------------------------------------------------------------------
  // 13) Stamp SSN output
  //
  // y = (u - Vc) / Rc
  // -------------------------------------------------------------------------
  C.block(0, VcA, 3, 3) = -1.0 / mRc * identity3;
  D = 1.0 / mRc * identity3;
}

Bool EMT::Ph3::AvVoltSourceInverterStateSpace::updateComponentParameters() {
  Matrix E;
  Matrix F;

  // The local linearized SSN model is time-varying because the network abc
  // states are coupled with dq-frame control through the Park transformation.
  // Therefore the stamp is recomputed every step and the change check is
  // intentionally skipped.
  buildStateSpaceModel(**mX, **mIntfVoltage, mA, mB, mC, mD, E, F);

  setStateOffset(E);
  setOutputOffset(F);

  return true;
}
