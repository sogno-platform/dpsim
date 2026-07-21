// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <stdexcept>

#include <dpsim-models/DP/DP_Ph3_AvVoltSourceInverterStateSpace.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

namespace {
const Real K23 = std::sqrt(2.0 / 3.0);
const Real K32 = std::sqrt(1.5);
} // namespace

DP::Ph3::AvVoltSourceInverterStateSpace::AvVoltSourceInverterStateSpace(
    String uid, String name, Logger::Level logLevel)
    : MixedVTypeVariableSSNComp(uid, name, 10, 6, logLevel), mLf(0.0), mCf(0.0),
      mRf(0.0), mRc(0.0), mOmegaN(0.0), mKpPLL(0.0), mKiPLL(0.0),
      mOmegaCutoff(0.0), mPRef(0.0), mQRef(0.0), mKpPowerCtrl(0.0),
      mKiPowerCtrl(0.0), mKpCurrCtrl(0.0), mKiCurrCtrl(0.0), mIRefNd(0.0),
      mIRefNq(0.0), mVcD(mAttributes->create<Real>("vc_d")),
      mVcQ(mAttributes->create<Real>("vc_q")),
      mIrcD(mAttributes->create<Real>("irc_d")),
      mIrcQ(mAttributes->create<Real>("irc_q")),
      mPInst(mAttributes->create<Real>("p_inst")),
      mQInst(mAttributes->create<Real>("q_inst")),
      mOmegaPLL(mAttributes->create<Real>("omega_pll")),
      mIrcNd(mAttributes->create<Real>("irc_n_d")),
      mIrcNq(mAttributes->create<Real>("irc_n_q")) {
  **mVcD = 0.0;
  **mVcQ = 0.0;
  **mIrcD = 0.0;
  **mIrcQ = 0.0;
  **mPInst = 0.0;
  **mQInst = 0.0;
  **mOmegaPLL = 0.0;
  **mIrcNd = 0.0;
  **mIrcNq = 0.0;
}

void DP::Ph3::AvVoltSourceInverterStateSpace::setParameters(
    Real lf, Real cf, Real rf, Real rc, Real omegaN, Real kpPLL, Real kiPLL,
    Real omegaCutoff, Real pRef, Real qRef, Real kpPowerCtrl, Real kiPowerCtrl,
    Real kpCurrCtrl, Real kiCurrCtrl, Real iRefNd, Real iRefNq) {
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
  mIRefNd = iRefNd;
  mIRefNq = iRefNq;

  const Matrix x0 = Matrix::Zero(stateSize(), 1);
  const Matrix u0 = Matrix::Zero(6, 1);

  Matrix aMatrix, bMatrix, cMatrix, dMatrix, eVector, fVector;
  buildStateSpaceModel(x0, u0, aMatrix, bMatrix, cMatrix, dMatrix, eVector,
                       fVector);

  MixedVTypeVariableSSNComp::setParameters(aMatrix, bMatrix, cMatrix, dMatrix,
                                           eVector, fVector);
}

void DP::Ph3::AvVoltSourceInverterStateSpace::buildStateSpaceModel(
    const Matrix &x, const Matrix &u, Matrix &A, Matrix &B, Matrix &C,
    Matrix &D, Matrix &E, Matrix &F) const {
  // Unpack the operating point. psi := theta0 - thetaN (never a state itself).
  const Real psi = x(Psi, 0);
  const Real phiPLL = x(PhiPLL, 0);
  const Real pF = x(PFiltered, 0);
  const Real qF = x(QFiltered, 0);
  const Real phiD = x(PhiD, 0);
  const Real phiQ = x(PhiQ, 0);
  const Real gammaD = x(GammaD, 0);
  const Real gammaQ = x(GammaQ, 0);
  const Real gammaND = x(GammaND, 0);
  const Real gammaNQ = x(GammaNQ, 0);

  const Complex j(0.0, 1.0);

  const Complex vc[3] = {Complex(x(VcARe, 0), x(VcAIm, 0)),
                         Complex(x(VcBRe, 0), x(VcBIm, 0)),
                         Complex(x(VcCRe, 0), x(VcCIm, 0))};
  const Complex iF[3] = {Complex(x(IfARe, 0), x(IfAIm, 0)),
                         Complex(x(IfBRe, 0), x(IfBIm, 0)),
                         Complex(x(IfCRe, 0), x(IfCIm, 0))};
  const Complex uEnv[3] = {Complex(u(0, 0), u(1, 0)), Complex(u(2, 0), u(3, 0)),
                           Complex(u(4, 0), u(5, 0))};

  // Positive-sequence projection coefficients and inverse-Park redistribution factors.
  const Complex projCoeff[3] = {Complex(1.0, 0.0), SHIFT_TO_PHASE_C,
                                SHIFT_TO_PHASE_B};
  const Complex redistFactor[3] = {std::conj(projCoeff[0]),
                                   std::conj(projCoeff[1]),
                                   std::conj(projCoeff[2])};

  const Complex rot = std::exp(-j * psi);
  const Complex expJPsi = std::conj(rot);

  // Positive-sequence dq measurements from pV = vc_a + a*vc_b + a^2*vc_c.
  Complex pV(0.0, 0.0), pU(0.0, 0.0);
  for (Int p = 0; p < 3; ++p) {
    pV += projCoeff[p] * vc[p];
    pU += projCoeff[p] * uEnv[p];
  }
  const Complex pI = (pV - pU) / mRc;

  const Complex vcDQ = 0.5 * K23 * rot * pV;
  const Complex ircDQ = 0.5 * K23 * rot * pI;
  const Real vcD = vcDQ.real();
  const Real vcQ = vcDQ.imag();
  const Real ircD = ircDQ.real();
  const Real ircQ = ircDQ.imag();

  // gVc[p] = d(vcDQ)/d(vc_p); gIrcVc/gIrcU = d(ircDQ)/d(vc_p), d(ircDQ)/d(u_p).
  Complex gVc[3], gIrcVc[3], gIrcU[3];
  for (Int p = 0; p < 3; ++p) {
    gVc[p] = 0.5 * K23 * rot * projCoeff[p];
    gIrcVc[p] = gVc[p] / mRc;
    gIrcU[p] = -gVc[p] / mRc;
  }

  // Power: p+jq = vcDQ*conj(ircDQ); psi-independent, so no Psi column below.
  const Complex pq = vcDQ * std::conj(ircDQ);
  const Real pInst = pq.real();
  const Real qInst = pq.imag();

  // Outer power control and inner current control, single dq frame.
  const Real iRefD =
      -mKpPowerCtrl * pF + mKiPowerCtrl * phiD + mKpPowerCtrl * mPRef;
  const Real iRefQ =
      mKpPowerCtrl * qF + mKiPowerCtrl * phiQ - mKpPowerCtrl * mQRef;
  const Complex iRefDQ(iRefD, iRefQ);
  const Complex gammaDQ(gammaD, gammaQ);

  const Complex vRefDQ =
      -mKpCurrCtrl * ircDQ + mKiCurrCtrl * gammaDQ + mKpCurrCtrl * iRefDQ;

  // Positive-sequence bridge-voltage reference, distributed via the inverse Park.
  const Complex vRefEnv0 = K23 * vRefDQ * expJPsi;

  // Negative-sequence measurement: conjugate projection, rotate by e^{+j*psi}, conjugate -> baseband ircNDQ.
  Complex nV(0.0, 0.0), nU(0.0, 0.0);
  for (Int p = 0; p < 3; ++p) {
    nV += redistFactor[p] * vc[p];
    nU += redistFactor[p] * uEnv[p];
  }
  const Complex nI = (nV - nU) / mRc;
  const Complex ircNDQ = 0.5 * K23 * expJPsi * std::conj(nI);
  const Real ircND = ircNDQ.real();
  const Real ircNQ = ircNDQ.imag();

  // Coefficient of conj(vc_p) in ircNDQ; the conjugate flips the imaginary-part derivative signs vs the positive loop.
  Complex hIrcN[3];
  for (Int p = 0; p < 3; ++p)
    hIrcN[p] = 0.5 * K23 * expJPsi * projCoeff[p] / mRc;

  // PI current control (positive-loop gains reused), then inverse negative Park.
  const Real vRefNd =
      -mKpCurrCtrl * ircND + mKiCurrCtrl * gammaND + mKpCurrCtrl * mIRefNd;
  const Real vRefNq =
      -mKpCurrCtrl * ircNQ + mKiCurrCtrl * gammaNQ + mKpCurrCtrl * mIRefNq;
  const Complex vRefNDQ(vRefNd, vRefNq);
  const Complex vRefNEnv0 = K23 * std::conj(vRefNDQ) * expJPsi;

  // Total reference: positive (redistFactor) + sequence-orthogonal negative (projCoeff) injection.
  Complex vRef[3];
  for (Int p = 0; p < 3; ++p)
    vRef[p] = redistFactor[p] * vRefEnv0 + projCoeff[p] * vRefNEnv0;

  // RHS f(x,u) (x_dot = f(x,u)).
  Complex vcDot[3], ifDot[3];
  for (Int p = 0; p < 3; ++p) {
    vcDot[p] =
        iF[p] / mCf + (uEnv[p] - vc[p]) / (mCf * mRc) - j * mOmegaN * vc[p];
    ifDot[p] = (vRef[p] - vc[p] - mRf * iF[p]) / mLf - j * mOmegaN * iF[p];
  }

  Matrix f = Matrix::Zero(22, 1);
  f(Psi, 0) = mKpPLL * vcQ + mKiPLL * phiPLL;
  f(PhiPLL, 0) = vcQ;
  f(PFiltered, 0) = mOmegaCutoff * (pInst - pF);
  f(QFiltered, 0) = mOmegaCutoff * (qInst - qF);
  f(PhiD, 0) = mPRef - pF;
  f(PhiQ, 0) = qF - mQRef;
  f(GammaD, 0) = iRefD - ircD;
  f(GammaQ, 0) = iRefQ - ircQ;
  f(GammaND, 0) = mIRefNd - ircND;
  f(GammaNQ, 0) = mIRefNq - ircNQ;
  f(VcARe, 0) = vcDot[0].real();
  f(VcAIm, 0) = vcDot[0].imag();
  f(VcBRe, 0) = vcDot[1].real();
  f(VcBIm, 0) = vcDot[1].imag();
  f(VcCRe, 0) = vcDot[2].real();
  f(VcCIm, 0) = vcDot[2].imag();
  f(IfARe, 0) = ifDot[0].real();
  f(IfAIm, 0) = ifDot[0].imag();
  f(IfBRe, 0) = ifDot[1].real();
  f(IfBIm, 0) = ifDot[1].imag();
  f(IfCRe, 0) = ifDot[2].real();
  f(IfCIm, 0) = ifDot[2].imag();

  // Analytic Jacobian A = df/dx, B = df/du.
  A = Matrix::Zero(22, 22);
  B = Matrix::Zero(22, 6);

  // PLL rows: only vcQ feeds them; d(vcQ)/dpsi = -vcD (d(vcDQ)/dpsi = -j*vcDQ).
  A(Psi, Psi) = mKpPLL * (-vcD);
  A(Psi, PhiPLL) = mKiPLL;
  A(PhiPLL, Psi) = -vcD;
  for (Int p = 0; p < 3; ++p) {
    const Real dVcQdRe = gVc[p].imag();
    const Real dVcQdIm = gVc[p].real();
    A(Psi, mVcReCol[p]) = mKpPLL * dVcQdRe;
    A(Psi, mVcImCol[p]) = mKpPLL * dVcQdIm;
    A(PhiPLL, mVcReCol[p]) = dVcQdRe;
    A(PhiPLL, mVcImCol[p]) = dVcQdIm;
  }

  // Power-filter rows: d(p+jq)/dv = dvcDQ/dv*conj(ircDQ) + vcDQ*conj(dircDQ/dv). No Psi column.
  A(PFiltered, PFiltered) = -mOmegaCutoff;
  A(QFiltered, QFiltered) = -mOmegaCutoff;
  for (Int p = 0; p < 3; ++p) {
    const Complex dpqVcRe =
        gVc[p] * std::conj(ircDQ) + vcDQ * std::conj(gIrcVc[p]);
    const Complex dpqVcIm =
        (j * gVc[p]) * std::conj(ircDQ) + vcDQ * std::conj(j * gIrcVc[p]);
    A(PFiltered, mVcReCol[p]) = mOmegaCutoff * dpqVcRe.real();
    A(QFiltered, mVcReCol[p]) = mOmegaCutoff * dpqVcRe.imag();
    A(PFiltered, mVcImCol[p]) = mOmegaCutoff * dpqVcIm.real();
    A(QFiltered, mVcImCol[p]) = mOmegaCutoff * dpqVcIm.imag();

    const Complex dpqURe = vcDQ * std::conj(gIrcU[p]);
    const Complex dpqUIm = vcDQ * std::conj(j * gIrcU[p]);
    B(PFiltered, mUReCol[p]) = mOmegaCutoff * dpqURe.real();
    B(QFiltered, mUReCol[p]) = mOmegaCutoff * dpqURe.imag();
    B(PFiltered, mUImCol[p]) = mOmegaCutoff * dpqUIm.real();
    B(QFiltered, mUImCol[p]) = mOmegaCutoff * dpqUIm.imag();
  }

  A(PhiD, PFiltered) = -1.0;
  A(PhiQ, QFiltered) = 1.0;

  // Current-control integrators: gammaD_dot = iRefD - ircD, etc.
  A(GammaD, PFiltered) = -mKpPowerCtrl;
  A(GammaD, PhiD) = mKiPowerCtrl;
  A(GammaD, Psi) = -ircQ;
  A(GammaQ, QFiltered) = mKpPowerCtrl;
  A(GammaQ, PhiQ) = mKiPowerCtrl;
  A(GammaQ, Psi) = ircD;
  for (Int p = 0; p < 3; ++p) {
    const Real dIrcDdVcRe = gIrcVc[p].real();
    const Real dIrcDdVcIm = -gIrcVc[p].imag();
    const Real dIrcQdVcRe = gIrcVc[p].imag();
    const Real dIrcQdVcIm = gIrcVc[p].real();
    A(GammaD, mVcReCol[p]) = -dIrcDdVcRe;
    A(GammaD, mVcImCol[p]) = -dIrcDdVcIm;
    A(GammaQ, mVcReCol[p]) = -dIrcQdVcRe;
    A(GammaQ, mVcImCol[p]) = -dIrcQdVcIm;

    const Real dIrcDdURe = gIrcU[p].real();
    const Real dIrcDdUIm = -gIrcU[p].imag();
    const Real dIrcQdURe = gIrcU[p].imag();
    const Real dIrcQdUIm = gIrcU[p].real();
    B(GammaD, mUReCol[p]) = -dIrcDdURe;
    B(GammaD, mUImCol[p]) = -dIrcDdUIm;
    B(GammaQ, mUReCol[p]) = -dIrcQdURe;
    B(GammaQ, mUImCol[p]) = -dIrcQdUIm;
  }

  // Negative-sequence current integrators: gammaND_dot = iRefNd - ircND, etc.
  // psi enters via rotN = e^{+j*psi}: d(ircND)/dpsi = -ircNQ, d(ircNQ)/dpsi = ircND.
  A(GammaND, Psi) = ircNQ;
  A(GammaNQ, Psi) = -ircND;
  for (Int p = 0; p < 3; ++p) {
    A(GammaND, mVcReCol[p]) = -hIrcN[p].real();
    A(GammaND, mVcImCol[p]) = -hIrcN[p].imag();
    A(GammaNQ, mVcReCol[p]) = -hIrcN[p].imag();
    A(GammaNQ, mVcImCol[p]) = hIrcN[p].real();
    B(GammaND, mUReCol[p]) = hIrcN[p].real();
    B(GammaND, mUImCol[p]) = hIrcN[p].imag();
    B(GammaNQ, mUReCol[p]) = hIrcN[p].imag();
    B(GammaNQ, mUImCol[p]) = -hIrcN[p].real();
  }

  // Filter capacitor rows (Vc_dot), fully decoupled per phase.
  for (Int p = 0; p < 3; ++p) {
    const Int reRow = mVcReCol[p];
    const Int imRow = mVcImCol[p];
    A(reRow, reRow) = -1.0 / (mCf * mRc);
    A(reRow, imRow) = mOmegaN;
    A(imRow, reRow) = -mOmegaN;
    A(imRow, imRow) = -1.0 / (mCf * mRc);
    A(reRow, mIfReCol[p]) = 1.0 / mCf;
    A(imRow, mIfImCol[p]) = 1.0 / mCf;
    B(reRow, mUReCol[p]) = 1.0 / (mCf * mRc);
    B(imRow, mUImCol[p]) = 1.0 / (mCf * mRc);
  }

  // Filter inductor rows (If_dot): the per-phase coupling via vRef_p = redistFactor[p]*vRefEnv0.
  const Complex dVRefEnv0DPsi =
      j * K23 * expJPsi * (mKpCurrCtrl * ircDQ + vRefDQ);

  // d(vRefDQ)/dOwnVar for the six own-frame control states.
  const Complex dVRefEnv0DpF = K23 * expJPsi * (mKpCurrCtrl * (-mKpPowerCtrl));
  const Complex dVRefEnv0DqF = K23 * expJPsi * j * (mKpCurrCtrl * mKpPowerCtrl);
  const Complex dVRefEnv0DPhiD = K23 * expJPsi * (mKpCurrCtrl * mKiPowerCtrl);
  const Complex dVRefEnv0DPhiQ =
      K23 * expJPsi * j * (mKpCurrCtrl * mKiPowerCtrl);
  const Complex dVRefEnv0DGammaD = K23 * expJPsi * mKiCurrCtrl;
  const Complex dVRefEnv0DGammaQ = K23 * expJPsi * j * mKiCurrCtrl;

  const Int ownCols[7] = {Psi,  PFiltered, QFiltered, PhiD,
                          PhiQ, GammaD,    GammaQ};
  const Complex dVRefEnv0Own[7] = {
      dVRefEnv0DPsi,  dVRefEnv0DpF,     dVRefEnv0DqF,    dVRefEnv0DPhiD,
      dVRefEnv0DPhiQ, dVRefEnv0DGammaD, dVRefEnv0DGammaQ};

  // d(vRefDQ)/d(vc_p), d(vRefDQ)/d(u_p) via vRefDQ's -kpCurrCtrl*ircDQ term.
  Complex dVRefEnv0VcRe[3], dVRefEnv0VcIm[3], dVRefEnv0URe[3], dVRefEnv0UIm[3];
  for (Int p = 0; p < 3; ++p) {
    dVRefEnv0VcRe[p] = K23 * expJPsi * (-mKpCurrCtrl * gIrcVc[p]);
    dVRefEnv0VcIm[p] = K23 * expJPsi * (-mKpCurrCtrl * j * gIrcVc[p]);
    dVRefEnv0URe[p] = K23 * expJPsi * (-mKpCurrCtrl * gIrcU[p]);
    dVRefEnv0UIm[p] = K23 * expJPsi * (-mKpCurrCtrl * j * gIrcU[p]);
  }

  // Negative injection vRefNEnv0 = K23*conj(vRefNDQ)*expJPsi: derivatives wrt psi and the two negative-loop states.
  const Complex dVRefNEnv0DPsi =
      j * K23 * expJPsi *
      (mKpCurrCtrl * std::conj(ircNDQ) + std::conj(vRefNDQ));
  const Complex dVRefNEnv0DGammaND = K23 * expJPsi * mKiCurrCtrl;
  const Complex dVRefNEnv0DGammaNQ = -j * K23 * expJPsi * mKiCurrCtrl;

  // d(vRefNEnv0)/d(vc_p), d(u_p) via conj(vRefNDQ)'s -kpCurrCtrl*ircND/Q term (conjugate-of-conjugate).
  Complex dVRefNEnv0VcRe[3], dVRefNEnv0VcIm[3], dVRefNEnv0URe[3],
      dVRefNEnv0UIm[3];
  for (Int p = 0; p < 3; ++p) {
    const Complex base = mKpCurrCtrl * std::conj(hIrcN[p]) * K23 * expJPsi;
    dVRefNEnv0VcRe[p] = -base;
    dVRefNEnv0VcIm[p] = -j * base;
    dVRefNEnv0URe[p] = base;
    dVRefNEnv0UIm[p] = j * base;
  }

  for (Int pOut = 0; pOut < 3; ++pOut) {
    const Int reRow = mIfReCol[pOut];
    const Int imRow = mIfImCol[pOut];

    // Own-phase direct terms: -(vc_p + Rf*iF_p)/Lf - j*omegaN*iF_p.
    A(reRow, mVcReCol[pOut]) = -1.0 / mLf;
    A(imRow, mVcImCol[pOut]) = -1.0 / mLf;
    A(reRow, reRow) = -mRf / mLf;
    A(imRow, imRow) = -mRf / mLf;
    A(reRow, imRow) = mOmegaN;
    A(imRow, reRow) = -mOmegaN;

    // vRef_p coupling through the shared single-dq-frame control chain.
    for (Int k = 0; k < 7; ++k) {
      const Complex dVRef = redistFactor[pOut] * dVRefEnv0Own[k];
      A(reRow, ownCols[k]) += dVRef.real() / mLf;
      A(imRow, ownCols[k]) += dVRef.imag() / mLf;
    }
    for (Int pSrc = 0; pSrc < 3; ++pSrc) {
      const Complex dVRefVcRe = redistFactor[pOut] * dVRefEnv0VcRe[pSrc];
      const Complex dVRefVcIm = redistFactor[pOut] * dVRefEnv0VcIm[pSrc];
      A(reRow, mVcReCol[pSrc]) += dVRefVcRe.real() / mLf;
      A(imRow, mVcReCol[pSrc]) += dVRefVcRe.imag() / mLf;
      A(reRow, mVcImCol[pSrc]) += dVRefVcIm.real() / mLf;
      A(imRow, mVcImCol[pSrc]) += dVRefVcIm.imag() / mLf;

      const Complex dVRefURe = redistFactor[pOut] * dVRefEnv0URe[pSrc];
      const Complex dVRefUIm = redistFactor[pOut] * dVRefEnv0UIm[pSrc];
      B(reRow, mUReCol[pSrc]) += dVRefURe.real() / mLf;
      B(imRow, mUReCol[pSrc]) += dVRefURe.imag() / mLf;
      B(reRow, mUImCol[pSrc]) += dVRefUIm.real() / mLf;
      B(imRow, mUImCol[pSrc]) += dVRefUIm.imag() / mLf;
    }

    // Negative-sequence injection coupling: vRef_p += projCoeff[pOut]*vRefNEnv0.
    const Complex dNPsi = projCoeff[pOut] * dVRefNEnv0DPsi;
    A(reRow, Psi) += dNPsi.real() / mLf;
    A(imRow, Psi) += dNPsi.imag() / mLf;
    const Complex dNGammaND = projCoeff[pOut] * dVRefNEnv0DGammaND;
    A(reRow, GammaND) += dNGammaND.real() / mLf;
    A(imRow, GammaND) += dNGammaND.imag() / mLf;
    const Complex dNGammaNQ = projCoeff[pOut] * dVRefNEnv0DGammaNQ;
    A(reRow, GammaNQ) += dNGammaNQ.real() / mLf;
    A(imRow, GammaNQ) += dNGammaNQ.imag() / mLf;
    for (Int pSrc = 0; pSrc < 3; ++pSrc) {
      const Complex dNVcRe = projCoeff[pOut] * dVRefNEnv0VcRe[pSrc];
      const Complex dNVcIm = projCoeff[pOut] * dVRefNEnv0VcIm[pSrc];
      A(reRow, mVcReCol[pSrc]) += dNVcRe.real() / mLf;
      A(imRow, mVcReCol[pSrc]) += dNVcRe.imag() / mLf;
      A(reRow, mVcImCol[pSrc]) += dNVcIm.real() / mLf;
      A(imRow, mVcImCol[pSrc]) += dNVcIm.imag() / mLf;

      const Complex dNURe = projCoeff[pOut] * dVRefNEnv0URe[pSrc];
      const Complex dNUIm = projCoeff[pOut] * dVRefNEnv0UIm[pSrc];
      B(reRow, mUReCol[pSrc]) += dNURe.real() / mLf;
      B(imRow, mUReCol[pSrc]) += dNURe.imag() / mLf;
      B(reRow, mUImCol[pSrc]) += dNUIm.real() / mLf;
      B(imRow, mUImCol[pSrc]) += dNUIm.imag() / mLf;
    }
  }

  // Offset E = f(x,u) - A*x - B*u.
  E = f - A * x - B * u;

  // SSN output: y_p = (u_p - vc_p)/Rc (exact, no relinearization needed).
  C = Matrix::Zero(6, 22);
  for (Int p = 0; p < 3; ++p) {
    C(2 * p, mVcReCol[p]) = -1.0 / mRc;
    C(2 * p + 1, mVcImCol[p]) = -1.0 / mRc;
  }

  D = Matrix::Zero(6, 6);
  for (Int p = 0; p < 3; ++p) {
    D(2 * p, mUReCol[p]) = 1.0 / mRc;
    D(2 * p + 1, mUImCol[p]) = 1.0 / mRc;
  }

  F = Matrix::Zero(6, 1);
}

Bool DP::Ph3::AvVoltSourceInverterStateSpace::updateComponentParameters() {
  Matrix E;
  Matrix F;

  // Relinearized every step (mirrors EMT/Ph1); change-check intentionally skipped.
  buildStateSpaceModel(**mX, packComplex((**inputAttribute())), mA, mB, mC, mD,
                       E, F);

  setStateOffset(E);
  setOutputOffset(F);

  return true;
}

void DP::Ph3::AvVoltSourceInverterStateSpace::updateLogAttributes(
    const Matrix &u) const {
  const Matrix &x = **mX;

  const Real psi = x(Psi, 0);
  const Complex rot = std::exp(Complex(0.0, -psi));
  const Complex expJPsi = std::conj(rot);
  const Complex projCoeff[3] = {Complex(1.0, 0.0), SHIFT_TO_PHASE_C,
                                SHIFT_TO_PHASE_B};

  Complex pV(0.0, 0.0), pU(0.0, 0.0), nV(0.0, 0.0), nU(0.0, 0.0);
  for (Int p = 0; p < 3; ++p) {
    const Complex vc(x(mVcReCol[p], 0), x(mVcImCol[p], 0));
    const Complex uEnv(u(mUReCol[p], 0), u(mUImCol[p], 0));
    pV += projCoeff[p] * vc;
    pU += projCoeff[p] * uEnv;
    nV += std::conj(projCoeff[p]) * vc;
    nU += std::conj(projCoeff[p]) * uEnv;
  }
  const Complex pI = (pV - pU) / mRc;
  const Complex nI = (nV - nU) / mRc;

  const Complex vcDQ = 0.5 * K23 * rot * pV;
  const Complex ircDQ = 0.5 * K23 * rot * pI;
  const Complex ircNDQ = 0.5 * K23 * expJPsi * std::conj(nI);

  **mVcD = vcDQ.real();
  **mVcQ = vcDQ.imag();
  **mIrcD = ircDQ.real();
  **mIrcQ = ircDQ.imag();
  **mIrcNd = ircNDQ.real();
  **mIrcNq = ircNDQ.imag();

  **mPInst = **mVcD * **mIrcD + **mVcQ * **mIrcQ;
  **mQInst = -**mVcD * **mIrcQ + **mVcQ * **mIrcD;

  **mOmegaPLL = mOmegaN + mKpPLL * **mVcQ + mKiPLL * x(PhiPLL, 0);
}

void DP::Ph3::AvVoltSourceInverterStateSpace::initializeFromNodesAndTerminals(
    Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  // Balanced init: solve the phase-a operating point, replicate across phases.
  const Real omega = 2.0 * PI * frequency;
  const Complex j(0.0, 1.0);
  const Complex powerRef(mPRef, mQRef);

  const MatrixComp uInit = buildInitialInputFromNodes(frequency);
  const Complex ua = uInit(0, 0);

  Complex vc = ua;
  Complex irc(0.0, 0.0);

  for (Int iter = 0; iter < mInitializationMaxIterations; ++iter) {
    if (std::abs(vc) < mInitializationTolerance) {
      irc = Complex(0.0, 0.0);
      break;
    }

    const Complex iNext = std::conj(powerRef / (1.5 * vc));
    const Complex vcNext = ua + mRc * iNext;

    irc = iNext;

    if (std::abs(vcNext - vc) < mInitializationTolerance) {
      vc = vcNext;
      break;
    }

    vc = vcNext;
  }

  const Complex ifCurrent = j * omega * mCf * vc + irc;
  const Complex vRef = vc + (mRf + j * omega * mLf) * ifCurrent;

  // thetaN(0) = 0, so psi0 = theta0 exactly.
  const Real psi0 = std::arg(vc);
  const Complex rot0 = std::exp(-j * psi0);

  const Complex vcDQ = K32 * vc * rot0;
  const Real vcD = vcDQ.real();
  const Real vcQ = vcDQ.imag();
  const Complex ircDQ = K32 * irc * rot0;
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

  const Complex vRefDQ0 = K32 * vRef * rot0;
  const Real gammaD0 =
      (vRefDQ0.real() + mKpCurrCtrl * (ircD - iRefD0)) / mKiCurrCtrl;
  const Real gammaQ0 =
      (vRefDQ0.imag() + mKpCurrCtrl * (ircQ - iRefQ0)) / mKiCurrCtrl;

  const MatrixComp vcAbc = Math::singlePhaseVariableToThreePhase(vc);
  const MatrixComp ifAbc = Math::singlePhaseVariableToThreePhase(ifCurrent);

  Matrix x0 = Matrix::Zero(stateSize(), 1);
  x0(Psi, 0) = psi0;
  x0(PhiPLL, 0) = phiPLL0;
  x0(PFiltered, 0) = pInit;
  x0(QFiltered, 0) = qInit;
  x0(PhiD, 0) = phiD0;
  x0(PhiQ, 0) = phiQ0;
  x0(GammaD, 0) = gammaD0;
  x0(GammaQ, 0) = gammaQ0;
  // Balanced start: no negative sequence, so the negative-loop integrators seed at zero.
  x0(GammaND, 0) = 0.0;
  x0(GammaNQ, 0) = 0.0;

  for (Int p = 0; p < 3; ++p) {
    x0(mVcReCol[p], 0) = vcAbc(p, 0).real();
    x0(mVcImCol[p], 0) = vcAbc(p, 0).imag();
    x0(mIfReCol[p], 0) = ifAbc(p, 0).real();
    x0(mIfImCol[p], 0) = ifAbc(p, 0).imag();
  }

  **mX = x0;
  **mIntfVoltage = uInit;
  (**mIntfCurrent)(0, 0) = (ua - vc) / mRc;
  (**mIntfCurrent)(1, 0) = (uInit(1, 0) - vcAbc(1, 0)) / mRc;
  (**mIntfCurrent)(2, 0) = (uInit(2, 0) - vcAbc(2, 0)) / mRc;

  updateComponentParameters();
  updateLogAttributes(packComplex(uInit));

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Inverter SSN mixed real+per-phase-complex "
                     "initialization ---"
                     "\nInput u: {:s}"
                     "\nOutput y: {:s}"
                     "\nState x: {:s}"
                     "\nP/Q init: [{:.6e}, {:.6e}]"
                     "\nVc dq (phase a): [{:.6e}, {:.6e}]"
                     "\nIinj dq (phase a): [{:.6e}, {:.6e}]"
                     "\n--- Initialization finished ---",
                     Logger::matrixCompToString(**mIntfVoltage),
                     Logger::matrixCompToString(**mIntfCurrent),
                     Logger::matrixToString(**mX), pInit, qInit, vcD, vcQ, ircD,
                     ircQ);
}
