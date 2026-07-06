// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <stdexcept>

#include <dpsim-models/DP/DP_Ph1_AvVoltSourceInverterStateSpace.h>

using namespace CPS;

DP::Ph1::AvVoltSourceInverterStateSpace::AvVoltSourceInverterStateSpace(
    String uid, String name, Logger::Level logLevel)
    : MixedVTypeVariableSSNComp(uid, name, 8, 2, logLevel), mLf(0.0), mCf(0.0),
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
  **mVcD = 0.0;
  **mVcQ = 0.0;
  **mIrcD = 0.0;
  **mIrcQ = 0.0;
  **mPInst = 0.0;
  **mQInst = 0.0;
  **mOmegaPLL = 0.0;
}

void DP::Ph1::AvVoltSourceInverterStateSpace::setParameters(
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

  const Matrix x0 = Matrix::Zero(12, 1);
  const Matrix u0 = Matrix::Zero(2, 1);

  Matrix aMatrix, bMatrix, cMatrix, dMatrix, eVector, fVector;
  buildStateSpaceModel(x0, u0, aMatrix, bMatrix, cMatrix, dMatrix, eVector,
                       fVector);

  MixedVTypeVariableSSNComp::setParameters(aMatrix, bMatrix, cMatrix, dMatrix,
                                           eVector, fVector);
}

void DP::Ph1::AvVoltSourceInverterStateSpace::buildStateSpaceModel(
    const Matrix &x, const Matrix &u, Matrix &A, Matrix &B, Matrix &C,
    Matrix &D, Matrix &E, Matrix &F) const {
  // 1) Unpack the operating point. psi := theta0 - thetaN (never a state itself).
  const Real psi = x(Psi, 0);
  const Real pF = x(PFiltered, 0);
  const Real qF = x(QFiltered, 0);
  const Real phiD = x(PhiD, 0);
  const Real phiQ = x(PhiQ, 0);
  const Real gammaD = x(GammaD, 0);
  const Real gammaQ = x(GammaQ, 0);
  const Real vcRe = x(VcRe, 0);
  const Real vcIm = x(VcIm, 0);
  const Real ifRe = x(IfRe, 0);
  const Real ifIm = x(IfIm, 0);

  const Real uRe = u(0, 0);
  const Real uIm = u(1, 0);

  const Real cosPsi = std::cos(psi);
  const Real sinPsi = std::sin(psi);

  // 2) dq measurements: Vc*e^{-j*psi} and Irc*e^{-j*psi}, Irc = (Vc-U)/Rc.
  const Real vcD = vcRe * cosPsi + vcIm * sinPsi;
  const Real vcQ = vcIm * cosPsi - vcRe * sinPsi;

  const Real ircRe = (vcRe - uRe) / mRc;
  const Real ircIm = (vcIm - uIm) / mRc;
  const Real ircD = ircRe * cosPsi + ircIm * sinPsi;
  const Real ircQ = ircIm * cosPsi - ircRe * sinPsi;

  // Jacobian of vcQ (only vq feeds the PLL rows).
  const Real dVcQByPsi = -vcIm * sinPsi - vcRe * cosPsi;
  const Real dVcQByVcRe = -sinPsi;
  const Real dVcQByVcIm = cosPsi;

  // Jacobian of ircD, ircQ.
  const Real dIrcDByPsi = -ircRe * sinPsi + ircIm * cosPsi;
  const Real dIrcDByVcRe = cosPsi / mRc;
  const Real dIrcDByVcIm = sinPsi / mRc;
  const Real dIrcDByURe = -cosPsi / mRc;
  const Real dIrcDByUIm = -sinPsi / mRc;

  const Real dIrcQByPsi = -ircIm * sinPsi - ircRe * cosPsi;
  const Real dIrcQByVcRe = -sinPsi / mRc;
  const Real dIrcQByVcIm = cosPsi / mRc;
  const Real dIrcQByURe = sinPsi / mRc;
  const Real dIrcQByUIm = -cosPsi / mRc;

  // 3) Power: P+jQ = (1/Rc)*Vc*conj(Vc-U), evaluated as the dot product below.
  const Real pInst = vcD * ircD + vcQ * ircQ;
  const Real qInst = -vcD * ircQ + vcQ * ircD;

  const Real dPByVcRe = (2.0 / mRc) * vcRe - (1.0 / mRc) * uRe;
  const Real dPByVcIm = (2.0 / mRc) * vcIm - (1.0 / mRc) * uIm;
  const Real dPByURe = -(1.0 / mRc) * vcRe;
  const Real dPByUIm = -(1.0 / mRc) * vcIm;

  const Real dQByVcRe = (1.0 / mRc) * uIm;
  const Real dQByVcIm = -(1.0 / mRc) * uRe;
  const Real dQByURe = -(1.0 / mRc) * vcIm;
  const Real dQByUIm = (1.0 / mRc) * vcRe;

  // 4) Outer power control and inner current control (dq).
  const Real iRefD =
      -mKpPowerCtrl * pF + mKiPowerCtrl * phiD + mKpPowerCtrl * mPRef;
  const Real iRefQ =
      mKpPowerCtrl * qF + mKiPowerCtrl * phiQ - mKpPowerCtrl * mQRef;

  const Real vRefD =
      -mKpCurrCtrl * ircD + mKiCurrCtrl * gammaD + mKpCurrCtrl * iRefD;
  const Real vRefQ =
      -mKpCurrCtrl * ircQ + mKiCurrCtrl * gammaQ + mKpCurrCtrl * iRefQ;

  // d(vRefD)/d{...}, d(vRefQ)/d{...} via the chain rule through ircD/ircQ, iRefD/iRefQ.
  const Real dVRefDByPsi = -mKpCurrCtrl * dIrcDByPsi;
  const Real dVRefDByVcRe = -mKpCurrCtrl * dIrcDByVcRe;
  const Real dVRefDByVcIm = -mKpCurrCtrl * dIrcDByVcIm;
  const Real dVRefDByURe = -mKpCurrCtrl * dIrcDByURe;
  const Real dVRefDByUIm = -mKpCurrCtrl * dIrcDByUIm;
  const Real dVRefDByPF = -mKpCurrCtrl * mKpPowerCtrl;
  const Real dVRefDByPhiD = mKpCurrCtrl * mKiPowerCtrl;
  const Real dVRefDByGammaD = mKiCurrCtrl;

  const Real dVRefQByPsi = -mKpCurrCtrl * dIrcQByPsi;
  const Real dVRefQByVcRe = -mKpCurrCtrl * dIrcQByVcRe;
  const Real dVRefQByVcIm = -mKpCurrCtrl * dIrcQByVcIm;
  const Real dVRefQByURe = -mKpCurrCtrl * dIrcQByURe;
  const Real dVRefQByUIm = -mKpCurrCtrl * dIrcQByUIm;
  const Real dVRefQByQF = mKpCurrCtrl * mKpPowerCtrl;
  const Real dVRefQByPhiQ = mKpCurrCtrl * mKiPowerCtrl;
  const Real dVRefQByGammaQ = mKiCurrCtrl;

  // 5) Bridge-voltage reference: VRefEnv = (vRefD+j*vRefQ)*e^{j*psi}; psi's partial needs the product rule, every other variable only enters implicitly.
  const Real vRefEnvRe = vRefD * cosPsi - vRefQ * sinPsi;
  const Real vRefEnvIm = vRefD * sinPsi + vRefQ * cosPsi;

  const Real dVRefEnvReByPsi = dVRefDByPsi * cosPsi - vRefD * sinPsi -
                               dVRefQByPsi * sinPsi - vRefQ * cosPsi;
  const Real dVRefEnvImByPsi = dVRefDByPsi * sinPsi + vRefD * cosPsi +
                               dVRefQByPsi * cosPsi - vRefQ * sinPsi;

  auto dVRefEnvRe = [&](Real dD, Real dQ) { return dD * cosPsi - dQ * sinPsi; };
  auto dVRefEnvIm = [&](Real dD, Real dQ) { return dD * sinPsi + dQ * cosPsi; };

  const Real dVRefEnvReByPF = dVRefEnvRe(dVRefDByPF, 0.0);
  const Real dVRefEnvReByQF = dVRefEnvRe(0.0, dVRefQByQF);
  const Real dVRefEnvReByPhiD = dVRefEnvRe(dVRefDByPhiD, 0.0);
  const Real dVRefEnvReByPhiQ = dVRefEnvRe(0.0, dVRefQByPhiQ);
  const Real dVRefEnvReByGammaD = dVRefEnvRe(dVRefDByGammaD, 0.0);
  const Real dVRefEnvReByGammaQ = dVRefEnvRe(0.0, dVRefQByGammaQ);
  const Real dVRefEnvReByVcRe = dVRefEnvRe(dVRefDByVcRe, dVRefQByVcRe);
  const Real dVRefEnvReByVcIm = dVRefEnvRe(dVRefDByVcIm, dVRefQByVcIm);
  const Real dVRefEnvReByURe = dVRefEnvRe(dVRefDByURe, dVRefQByURe);
  const Real dVRefEnvReByUIm = dVRefEnvRe(dVRefDByUIm, dVRefQByUIm);

  const Real dVRefEnvImByPF = dVRefEnvIm(dVRefDByPF, 0.0);
  const Real dVRefEnvImByQF = dVRefEnvIm(0.0, dVRefQByQF);
  const Real dVRefEnvImByPhiD = dVRefEnvIm(dVRefDByPhiD, 0.0);
  const Real dVRefEnvImByPhiQ = dVRefEnvIm(0.0, dVRefQByPhiQ);
  const Real dVRefEnvImByGammaD = dVRefEnvIm(dVRefDByGammaD, 0.0);
  const Real dVRefEnvImByGammaQ = dVRefEnvIm(0.0, dVRefQByGammaQ);
  const Real dVRefEnvImByVcRe = dVRefEnvIm(dVRefDByVcRe, dVRefQByVcRe);
  const Real dVRefEnvImByVcIm = dVRefEnvIm(dVRefDByVcIm, dVRefQByVcIm);
  const Real dVRefEnvImByURe = dVRefEnvIm(dVRefDByURe, dVRefQByURe);
  const Real dVRefEnvImByUIm = dVRefEnvIm(dVRefDByUIm, dVRefQByUIm);

  // 6) RHS f(x,u) (x_dot = f(x,u)).
  Matrix f = Matrix::Zero(12, 1);
  f(Psi, 0) = mKpPLL * vcQ + mKiPLL * x(PhiPLL, 0);
  f(PhiPLL, 0) = vcQ;
  f(PFiltered, 0) = mOmegaCutoff * (pInst - pF);
  f(QFiltered, 0) = mOmegaCutoff * (qInst - qF);
  f(PhiD, 0) = mPRef - pF;
  f(PhiQ, 0) = qF - mQRef;
  f(GammaD, 0) = iRefD - ircD;
  f(GammaQ, 0) = iRefQ - ircQ;
  f(VcRe, 0) = ifRe / mCf + (uRe - vcRe) / (mCf * mRc) + mOmegaN * vcIm;
  f(VcIm, 0) = ifIm / mCf + (uIm - vcIm) / (mCf * mRc) - mOmegaN * vcRe;
  f(IfRe, 0) = (vRefEnvRe - vcRe - mRf * ifRe) / mLf + mOmegaN * ifIm;
  f(IfIm, 0) = (vRefEnvIm - vcIm - mRf * ifIm) / mLf - mOmegaN * ifRe;

  // 7) Analytic Jacobian A = df/dx, B = df/du.
  A = Matrix::Zero(12, 12);
  B = Matrix::Zero(12, 2);

  A(Psi, Psi) = mKpPLL * dVcQByPsi;
  A(Psi, PhiPLL) = mKiPLL;
  A(Psi, VcRe) = mKpPLL * dVcQByVcRe;
  A(Psi, VcIm) = mKpPLL * dVcQByVcIm;

  A(PhiPLL, Psi) = dVcQByPsi;
  A(PhiPLL, VcRe) = dVcQByVcRe;
  A(PhiPLL, VcIm) = dVcQByVcIm;

  A(PFiltered, PFiltered) = -mOmegaCutoff;
  A(PFiltered, VcRe) = mOmegaCutoff * dPByVcRe;
  A(PFiltered, VcIm) = mOmegaCutoff * dPByVcIm;
  B(PFiltered, 0) = mOmegaCutoff * dPByURe;
  B(PFiltered, 1) = mOmegaCutoff * dPByUIm;

  A(QFiltered, QFiltered) = -mOmegaCutoff;
  A(QFiltered, VcRe) = mOmegaCutoff * dQByVcRe;
  A(QFiltered, VcIm) = mOmegaCutoff * dQByVcIm;
  B(QFiltered, 0) = mOmegaCutoff * dQByURe;
  B(QFiltered, 1) = mOmegaCutoff * dQByUIm;

  A(PhiD, PFiltered) = -1.0;
  A(PhiQ, QFiltered) = 1.0;

  A(GammaD, PFiltered) = -mKpPowerCtrl;
  A(GammaD, PhiD) = mKiPowerCtrl;
  A(GammaD, Psi) = -dIrcDByPsi;
  A(GammaD, VcRe) = -dIrcDByVcRe;
  A(GammaD, VcIm) = -dIrcDByVcIm;
  B(GammaD, 0) = -dIrcDByURe;
  B(GammaD, 1) = -dIrcDByUIm;

  A(GammaQ, QFiltered) = mKpPowerCtrl;
  A(GammaQ, PhiQ) = mKiPowerCtrl;
  A(GammaQ, Psi) = -dIrcQByPsi;
  A(GammaQ, VcRe) = -dIrcQByVcRe;
  A(GammaQ, VcIm) = -dIrcQByVcIm;
  B(GammaQ, 0) = -dIrcQByURe;
  B(GammaQ, 1) = -dIrcQByUIm;

  A(VcRe, VcRe) = -1.0 / (mCf * mRc);
  A(VcRe, VcIm) = mOmegaN;
  A(VcRe, IfRe) = 1.0 / mCf;
  B(VcRe, 0) = 1.0 / (mCf * mRc);

  A(VcIm, VcRe) = -mOmegaN;
  A(VcIm, VcIm) = -1.0 / (mCf * mRc);
  A(VcIm, IfIm) = 1.0 / mCf;
  B(VcIm, 1) = 1.0 / (mCf * mRc);

  A(IfRe, Psi) = dVRefEnvReByPsi / mLf;
  A(IfRe, PFiltered) = dVRefEnvReByPF / mLf;
  A(IfRe, QFiltered) = dVRefEnvReByQF / mLf;
  A(IfRe, PhiD) = dVRefEnvReByPhiD / mLf;
  A(IfRe, PhiQ) = dVRefEnvReByPhiQ / mLf;
  A(IfRe, GammaD) = dVRefEnvReByGammaD / mLf;
  A(IfRe, GammaQ) = dVRefEnvReByGammaQ / mLf;
  A(IfRe, VcRe) = dVRefEnvReByVcRe / mLf - 1.0 / mLf;
  A(IfRe, VcIm) = dVRefEnvReByVcIm / mLf;
  A(IfRe, IfRe) = -mRf / mLf;
  A(IfRe, IfIm) = mOmegaN;
  B(IfRe, 0) = dVRefEnvReByURe / mLf;
  B(IfRe, 1) = dVRefEnvReByUIm / mLf;

  A(IfIm, Psi) = dVRefEnvImByPsi / mLf;
  A(IfIm, PFiltered) = dVRefEnvImByPF / mLf;
  A(IfIm, QFiltered) = dVRefEnvImByQF / mLf;
  A(IfIm, PhiD) = dVRefEnvImByPhiD / mLf;
  A(IfIm, PhiQ) = dVRefEnvImByPhiQ / mLf;
  A(IfIm, GammaD) = dVRefEnvImByGammaD / mLf;
  A(IfIm, GammaQ) = dVRefEnvImByGammaQ / mLf;
  A(IfIm, VcRe) = dVRefEnvImByVcRe / mLf;
  A(IfIm, VcIm) = dVRefEnvImByVcIm / mLf - 1.0 / mLf;
  A(IfIm, IfRe) = -mOmegaN;
  A(IfIm, IfIm) = -mRf / mLf;
  B(IfIm, 0) = dVRefEnvImByURe / mLf;
  B(IfIm, 1) = dVRefEnvImByUIm / mLf;

  // 8) Offset E = f(x,u) - A*x - B*u.
  E = f - A * x - B * u;

  // 9) SSN output: y = (u - Vc)/Rc (exact, no relinearization needed).
  C = Matrix::Zero(2, 12);
  C(0, VcRe) = -1.0 / mRc;
  C(1, VcIm) = -1.0 / mRc;

  D = Matrix::Zero(2, 2);
  D(0, 0) = 1.0 / mRc;
  D(1, 1) = 1.0 / mRc;

  F = Matrix::Zero(2, 1);
}

Bool DP::Ph1::AvVoltSourceInverterStateSpace::updateComponentParameters() {
  Matrix E;
  Matrix F;

  // Relinearized every step (mirrors EMT); change-check intentionally skipped.
  buildStateSpaceModel(**mX, packComplex((**inputAttribute())(0, 0)), mA, mB,
                       mC, mD, E, F);

  setStateOffset(E);
  setOutputOffset(F);

  return true;
}

void DP::Ph1::AvVoltSourceInverterStateSpace::updateLogAttributes(
    const Matrix &u) const {
  const Matrix &x = **mX;

  const Real psi = x(Psi, 0);
  const Real cosPsi = std::cos(psi);
  const Real sinPsi = std::sin(psi);
  const Real vcRe = x(VcRe, 0);
  const Real vcIm = x(VcIm, 0);
  const Real ircRe = (vcRe - u(0, 0)) / mRc;
  const Real ircIm = (vcIm - u(1, 0)) / mRc;

  **mVcD = vcRe * cosPsi + vcIm * sinPsi;
  **mVcQ = vcIm * cosPsi - vcRe * sinPsi;
  **mIrcD = ircRe * cosPsi + ircIm * sinPsi;
  **mIrcQ = ircIm * cosPsi - ircRe * sinPsi;

  **mPInst = **mVcD * **mIrcD + **mVcQ * **mIrcQ;
  **mQInst = -**mVcD * **mIrcQ + **mVcQ * **mIrcD;

  **mOmegaPLL = mOmegaN + mKpPLL * **mVcQ + mKiPLL * x(PhiPLL, 0);
}

void DP::Ph1::AvVoltSourceInverterStateSpace::initializeFromNodesAndTerminals(
    Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  // Initialized algebraically (mixed state), mirroring EMT::Ph3::AvVoltSourceInverterStateSpace.

  const Real omega = 2.0 * PI * frequency;
  const Complex powerRef(mPRef, mQRef);

  const MatrixComp uInit = buildInitialInputFromNodes(frequency);
  const Complex U = uInit(0, 0);

  Complex vc = U;
  Complex irc(0.0, 0.0);

  for (Int iter = 0; iter < mInitializationMaxIterations; ++iter) {
    if (std::abs(vc) < mInitializationTolerance) {
      irc = Complex(0.0, 0.0);
      break;
    }

    const Complex iNext = std::conj(powerRef / vc);
    const Complex vcNext = U + mRc * iNext;

    irc = iNext;

    if (std::abs(vcNext - vc) < mInitializationTolerance) {
      vc = vcNext;
      break;
    }

    vc = vcNext;
  }

  const Complex j(0.0, 1.0);
  const Complex ifCurrent = j * omega * mCf * vc + irc;
  const Complex vRef = vc + (mRf + j * omega * mLf) * ifCurrent;

  // thetaN(0) = 0, so psi0 = theta0 exactly.
  const Real psi0 = std::arg(vc);
  const Complex rot0 = std::exp(-j * psi0);

  const Complex vcDQ = vc * rot0;
  const Real vcD = vcDQ.real();
  const Real vcQ = vcDQ.imag();
  const Complex ircDQ = irc * rot0;
  const Real ircD = ircDQ.real();
  const Real ircQ = ircDQ.imag();

  const Real pInit = vcD * ircD + vcQ * ircQ;
  const Real qInit = -vcD * ircQ + vcQ * ircD;

  const Real phiPLL0 = (omega - mOmegaN) / mKiPLL;
  const Real phiD0 = (ircD + mKpPowerCtrl * (pInit - mPRef)) / mKiPowerCtrl;
  const Real phiQ0 = (ircQ - mKpPowerCtrl * (qInit - mQRef)) / mKiPowerCtrl;

  const Real iRefD0 =
      -mKpPowerCtrl * pInit + mKiPowerCtrl * phiD0 + mKpPowerCtrl * mPRef;
  const Real iRefQ0 =
      mKpPowerCtrl * qInit + mKiPowerCtrl * phiQ0 - mKpPowerCtrl * mQRef;

  const Complex vRefDQ = vRef * rot0;
  const Real gammaD0 =
      (vRefDQ.real() + mKpCurrCtrl * (ircD - iRefD0)) / mKiCurrCtrl;
  const Real gammaQ0 =
      (vRefDQ.imag() + mKpCurrCtrl * (ircQ - iRefQ0)) / mKiCurrCtrl;

  Matrix x0 = Matrix::Zero(stateSize(), 1);
  x0(Psi, 0) = psi0;
  x0(PhiPLL, 0) = phiPLL0;
  x0(PFiltered, 0) = pInit;
  x0(QFiltered, 0) = qInit;
  x0(PhiD, 0) = phiD0;
  x0(PhiQ, 0) = phiQ0;
  x0(GammaD, 0) = gammaD0;
  x0(GammaQ, 0) = gammaQ0;
  x0(VcRe, 0) = vc.real();
  x0(VcIm, 0) = vc.imag();
  x0(IfRe, 0) = ifCurrent.real();
  x0(IfIm, 0) = ifCurrent.imag();

  **mX = x0;
  **mIntfVoltage = uInit;
  (**mIntfCurrent)(0, 0) = (U - vc) / mRc;

  updateComponentParameters();
  updateLogAttributes(packComplex(U));

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Inverter SSN mixed real+complex initialization ---"
                     "\nInput u: {:s}"
                     "\nOutput y: {:s}"
                     "\nState x: {:s}"
                     "\nP/Q init: [{:.6e}, {:.6e}]"
                     "\nVc dq: [{:.6e}, {:.6e}]"
                     "\nIinj dq: [{:.6e}, {:.6e}]"
                     "\n--- Initialization finished ---",
                     Logger::matrixCompToString(**mIntfVoltage),
                     Logger::matrixCompToString(**mIntfCurrent),
                     Logger::matrixToString(**mX), pInit, qInit, vcD, vcQ, ircD,
                     ircQ);
}
