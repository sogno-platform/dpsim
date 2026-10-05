// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <stdexcept>

#include <dpsim-models/DP/DP_Ph3_SSN_GFL_Split.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

namespace {
const Real K23 = std::sqrt(2.0 / 3.0);
const Real K32 = std::sqrt(1.5);
} // namespace

DP::Ph3::SSN_GFL_Split::SSN_GFL_Split(String uid, String name,
                                      Logger::Level logLevel)
    : TwoTerminalVTypeSplitSSNComp(uid, name, mControllerStateSize,
                                   mControllerInputSize, mControllerOutputSize,
                                   logLevel),
      mVcD(mAttributes->create<Real>("vc_d")),
      mVcQ(mAttributes->create<Real>("vc_q")),
      mIrcD(mAttributes->create<Real>("irc_d")),
      mIrcQ(mAttributes->create<Real>("irc_q")),
      mPInst(mAttributes->create<Real>("p_inst")),
      mQInst(mAttributes->create<Real>("q_inst")),
      mOmegaPLL(mAttributes->create<Real>("omega_pll")) {}

std::vector<String> DP::Ph3::SSN_GFL_Split::getSplitLocalStateNames() const {
  return {"psi",
          "phi_pll",
          "p_filtered",
          "q_filtered",
          "phi_d",
          "phi_q",
          "gamma_d",
          "gamma_q",
          "vc_a_re",
          "vc_a_im",
          "vc_b_re",
          "vc_b_im",
          "vc_c_re",
          "vc_c_im",
          "if_a_re",
          "if_a_im",
          "if_b_re",
          "if_b_im",
          "if_c_re",
          "if_c_im",
          "v_inv_delay_a_re",
          "v_inv_delay_a_im",
          "v_inv_delay_b_re",
          "v_inv_delay_b_im",
          "v_inv_delay_c_re",
          "v_inv_delay_c_im"};
}

void DP::Ph3::SSN_GFL_Split::setParameters(Real lf, Real cf, Real rf, Real rc,
                                           Real omegaN, Real kpPLL, Real kiPLL,
                                           Real omegaCutoff, Real pRef,
                                           Real qRef, Real kpPowerCtrl,
                                           Real kiPowerCtrl, Real kpCurrCtrl,
                                           Real kiCurrCtrl) {
  if (lf <= 0.0 || cf <= 0.0 || rc <= 0.0 || omegaN <= 0.0 || rf < 0.0 ||
      omegaCutoff < 0.0 || kiPLL == 0.0 || kiPowerCtrl == 0.0 ||
      kiCurrCtrl == 0.0)
    throw std::invalid_argument("Invalid DP split GFL parameters.");

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

  Matrix A = Matrix::Zero(mNetworkStateSize, mNetworkStateSize);
  Matrix B = Matrix::Zero(mNetworkStateSize, mTerminalInputSize);
  Matrix C = Matrix::Zero(mOutputSize, mNetworkStateSize);
  Matrix D = Matrix::Zero(mOutputSize, mTerminalInputSize);
  Matrix Bv = Matrix::Zero(mNetworkStateSize, mControllerOutputSize);

  for (Int p = 0; p < 3; ++p) {
    const Int vr = mVcRe[p], vi = mVcIm[p];
    const Int ir = mIfRe[p], ii = mIfIm[p];
    const Int ur = 2 * p, ui = 2 * p + 1;
    A(vr, vr) = A(vi, vi) = -1.0 / (mCf * mRc);
    A(vr, vi) = mOmegaN;
    A(vi, vr) = -mOmegaN;
    A(vr, ir) = A(vi, ii) = 1.0 / mCf;
    A(ir, vr) = A(ii, vi) = -1.0 / mLf;
    A(ir, ir) = A(ii, ii) = -mRf / mLf;
    A(ir, ii) = mOmegaN;
    A(ii, ir) = -mOmegaN;
    B(vr, ur) = B(vi, ui) = 1.0 / (mCf * mRc);
    Bv(ir, ur) = Bv(ii, ui) = 1.0 / mLf;
    C(ur, vr) = C(ui, vi) = -1.0 / mRc;
    D(ur, ur) = D(ui, ui) = 1.0 / mRc;
  }

  Matrix Hx = Matrix::Zero(mControllerInputSize, mNetworkStateSize);
  Matrix Hu = Matrix::Zero(mControllerInputSize, mTerminalInputSize);
  Hx.block(0, 0, 6, 6) = Matrix::Identity(6, 6);
  Hx.block(6, 0, 6, 6) = (1.0 / mRc) * Matrix::Identity(6, 6);
  Hu.block(6, 0, 6, 6) = (-1.0 / mRc) * Matrix::Identity(6, 6);
  setSplitParameters(A, B, C, D, Bv, Hx, Hu);
}

void DP::Ph3::SSN_GFL_Split::unpackMeasurement(const Matrix &measurement,
                                               Complex3 &vc,
                                               Complex3 &iGrid) const {
  for (Int p = 0; p < 3; ++p) {
    vc[p] = Complex(measurement(mMeasVcRe[p], 0), measurement(mMeasVcIm[p], 0));
    iGrid[p] =
        Complex(measurement(mMeasIRe[p], 0), measurement(mMeasIIm[p], 0));
  }
}

void DP::Ph3::SSN_GFL_Split::evaluateControllerStateDerivative(
    const Matrix &x, const Matrix &measurement, Matrix &f) const {
  Complex3 vc, iGrid;
  unpackMeasurement(measurement, vc, iGrid);
  const Complex3 projection = {Complex(1.0, 0.0), SHIFT_TO_PHASE_C,
                               SHIFT_TO_PHASE_B};
  Complex pV(0.0, 0.0), pI(0.0, 0.0);
  for (Int p = 0; p < 3; ++p) {
    pV += projection[p] * vc[p];
    pI += projection[p] * iGrid[p];
  }
  const Complex rot = std::exp(Complex(0.0, -x(Psi, 0)));
  const Complex vcDq = 0.5 * K23 * rot * pV;
  const Complex iDq = 0.5 * K23 * rot * pI;
  const Real pInst = (vcDq * std::conj(iDq)).real();
  const Real qInst = (vcDq * std::conj(iDq)).imag();
  const Real iRefD = -mKpPowerCtrl * x(PFiltered, 0) +
                     mKiPowerCtrl * x(PhiD, 0) + mKpPowerCtrl * mPRef;
  const Real iRefQ = mKpPowerCtrl * x(QFiltered, 0) +
                     mKiPowerCtrl * x(PhiQ, 0) - mKpPowerCtrl * mQRef;

  f = Matrix::Zero(mControllerStateSize, 1);
  f(Psi, 0) = mKpPLL * vcDq.imag() + mKiPLL * x(PhiPLL, 0);
  f(PhiPLL, 0) = vcDq.imag();
  f(PFiltered, 0) = mOmegaCutoff * (pInst - x(PFiltered, 0));
  f(QFiltered, 0) = mOmegaCutoff * (qInst - x(QFiltered, 0));
  f(PhiD, 0) = mPRef - x(PFiltered, 0);
  f(PhiQ, 0) = x(QFiltered, 0) - mQRef;
  f(GammaD, 0) = iRefD - iDq.real();
  f(GammaQ, 0) = iRefQ - iDq.imag();
}

void DP::Ph3::SSN_GFL_Split::evaluateControllerOutput(const Matrix &x,
                                                      const Matrix &measurement,
                                                      Matrix &output) const {
  Complex3 vc, iGrid;
  unpackMeasurement(measurement, vc, iGrid);
  const Complex3 projection = {Complex(1.0, 0.0), SHIFT_TO_PHASE_C,
                               SHIFT_TO_PHASE_B};
  Complex pI(0.0, 0.0);
  for (Int p = 0; p < 3; ++p)
    pI += projection[p] * iGrid[p];
  const Complex rot = std::exp(Complex(0.0, -x(Psi, 0)));
  const Complex expJPsi = std::conj(rot);
  const Complex iDq = 0.5 * K23 * rot * pI;
  const Complex iRef(-mKpPowerCtrl * x(PFiltered, 0) +
                         mKiPowerCtrl * x(PhiD, 0) + mKpPowerCtrl * mPRef,
                     mKpPowerCtrl * x(QFiltered, 0) +
                         mKiPowerCtrl * x(PhiQ, 0) - mKpPowerCtrl * mQRef);
  const Complex gamma(x(GammaD, 0), x(GammaQ, 0));
  const Complex vRefDq =
      -mKpCurrCtrl * iDq + mKiCurrCtrl * gamma + mKpCurrCtrl * iRef;
  const Complex vRefA = K23 * vRefDq * expJPsi;
  output = Matrix::Zero(6, 1);
  for (Int p = 0; p < 3; ++p) {
    const Complex value = std::conj(projection[p]) * vRefA;
    output(2 * p, 0) = value.real();
    output(2 * p + 1, 0) = value.imag();
  }
}

void DP::Ph3::SSN_GFL_Split::calculateControllerAnalyticalJacobians(
    const Matrix &x, const Matrix &measurement, Matrix &A, Matrix &B, Matrix &C,
    Matrix &D) const {
  Complex3 vc, iGrid;
  unpackMeasurement(measurement, vc, iGrid);
  const Complex j(0.0, 1.0);
  const Complex3 projection = {Complex(1.0, 0.0), SHIFT_TO_PHASE_C,
                               SHIFT_TO_PHASE_B};
  Complex pV(0.0, 0.0), pI(0.0, 0.0);
  for (Int p = 0; p < 3; ++p) {
    pV += projection[p] * vc[p];
    pI += projection[p] * iGrid[p];
  }
  const Complex rot = std::exp(-j * x(Psi, 0));
  const Complex expJPsi = std::conj(rot);
  const Complex vcDq = 0.5 * K23 * rot * pV;
  const Complex iDq = 0.5 * K23 * rot * pI;
  const Complex iRef(-mKpPowerCtrl * x(PFiltered, 0) +
                         mKiPowerCtrl * x(PhiD, 0) + mKpPowerCtrl * mPRef,
                     mKpPowerCtrl * x(QFiltered, 0) +
                         mKiPowerCtrl * x(PhiQ, 0) - mKpPowerCtrl * mQRef);
  const Complex gamma(x(GammaD, 0), x(GammaQ, 0));
  const Complex vRefDq =
      -mKpCurrCtrl * iDq + mKiCurrCtrl * gamma + mKpCurrCtrl * iRef;

  A = Matrix::Zero(8, 8);
  B = Matrix::Zero(8, 12);
  C = Matrix::Zero(6, 8);
  D = Matrix::Zero(6, 12);
  A(Psi, Psi) = -mKpPLL * vcDq.real();
  A(Psi, PhiPLL) = mKiPLL;
  A(PhiPLL, Psi) = -vcDq.real();
  A(PFiltered, PFiltered) = -mOmegaCutoff;
  A(QFiltered, QFiltered) = -mOmegaCutoff;
  A(PhiD, PFiltered) = -1.0;
  A(PhiQ, QFiltered) = 1.0;
  A(GammaD, PFiltered) = -mKpPowerCtrl;
  A(GammaD, PhiD) = mKiPowerCtrl;
  A(GammaD, Psi) = -iDq.imag();
  A(GammaQ, QFiltered) = mKpPowerCtrl;
  A(GammaQ, PhiQ) = mKiPowerCtrl;
  A(GammaQ, Psi) = iDq.real();

  for (Int p = 0; p < 3; ++p) {
    const Complex gv = 0.5 * K23 * rot * projection[p];
    const Complex gi = gv;
    const std::array<Complex, 2> dVc = {gv, j * gv};
    const std::array<Complex, 2> dI = {gi, j * gi};
    for (Int part = 0; part < 2; ++part) {
      const Int vcol = part == 0 ? mMeasVcRe[p] : mMeasVcIm[p];
      const Int icol = part == 0 ? mMeasIRe[p] : mMeasIIm[p];
      B(Psi, vcol) = mKpPLL * dVc[part].imag();
      B(PhiPLL, vcol) = dVc[part].imag();
      const Complex dpqV = dVc[part] * std::conj(iDq);
      const Complex dpqI = vcDq * std::conj(dI[part]);
      B(PFiltered, vcol) = mOmegaCutoff * dpqV.real();
      B(QFiltered, vcol) = mOmegaCutoff * dpqV.imag();
      B(PFiltered, icol) = mOmegaCutoff * dpqI.real();
      B(QFiltered, icol) = mOmegaCutoff * dpqI.imag();
      B(GammaD, icol) = -dI[part].real();
      B(GammaQ, icol) = -dI[part].imag();
    }
  }

  const Complex dVRefPsi = j * K23 * expJPsi * (mKpCurrCtrl * iDq + vRefDq);
  const std::array<Complex, 8> own = {
      dVRefPsi,
      Complex(0.0, 0.0),
      K23 * expJPsi * (-mKpCurrCtrl * mKpPowerCtrl),
      K23 * expJPsi * j * (mKpCurrCtrl * mKpPowerCtrl),
      K23 * expJPsi * (mKpCurrCtrl * mKiPowerCtrl),
      K23 * expJPsi * j * (mKpCurrCtrl * mKiPowerCtrl),
      K23 * expJPsi * mKiCurrCtrl,
      K23 * expJPsi * j * mKiCurrCtrl};

  for (Int pout = 0; pout < 3; ++pout) {
    const Complex redistribute = std::conj(projection[pout]);
    for (Int state = 0; state < 8; ++state) {
      const Complex value = redistribute * own[state];
      C(2 * pout, state) = value.real();
      C(2 * pout + 1, state) = value.imag();
    }
    for (Int pin = 0; pin < 3; ++pin) {
      const Complex gi = 0.5 * K23 * rot * projection[pin];
      const Complex dRe = redistribute * K23 * expJPsi * (-mKpCurrCtrl * gi);
      const Complex dIm =
          redistribute * K23 * expJPsi * (-mKpCurrCtrl * j * gi);
      D(2 * pout, mMeasIRe[pin]) = dRe.real();
      D(2 * pout + 1, mMeasIRe[pin]) = dRe.imag();
      D(2 * pout, mMeasIIm[pin]) = dIm.real();
      D(2 * pout + 1, mMeasIIm[pin]) = dIm.imag();
    }
  }
}

void DP::Ph3::SSN_GFL_Split::buildControllerStateSpaceModel(
    const Matrix &x, const Matrix &measurement, Matrix &A, Matrix &B, Matrix &C,
    Matrix &D, Matrix &E, Matrix &F) const {
  calculateControllerAnalyticalJacobians(x, measurement, A, B, C, D);
  Matrix derivative, output;
  evaluateControllerStateDerivative(x, measurement, derivative);
  evaluateControllerOutput(x, measurement, output);
  E = derivative - A * x - B * measurement;
  F = output - C * x - D * measurement;
}

void DP::Ph3::SSN_GFL_Split::initializeFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before initialization.");
  const Real omega = 2.0 * PI * frequency;
  const Complex j(0.0, 1.0);
  const MatrixComp u = buildInitialInputFromNodes(frequency);
  Complex vc = u(0, 0), iGrid(0.0, 0.0);
  for (Int iteration = 0; iteration < 10; ++iteration) {
    if (std::abs(vc) < 1e-9)
      break;
    const Complex nextI = std::conj(Complex(mPRef, mQRef) / (1.5 * vc));
    const Complex nextVc = u(0, 0) + mRc * nextI;
    iGrid = nextI;
    if (std::abs(nextVc - vc) < 1e-9) {
      vc = nextVc;
      break;
    }
    vc = nextVc;
  }
  const Complex ifCurrent = j * omega * mCf * vc + iGrid;
  const Complex vRef = vc + (mRf + j * omega * mLf) * ifCurrent;
  const MatrixComp vcAbc = Math::singlePhaseVariableToThreePhase(vc);
  const MatrixComp ifAbc = Math::singlePhaseVariableToThreePhase(ifCurrent);
  const MatrixComp iAbc = Math::singlePhaseVariableToThreePhase(iGrid);
  const MatrixComp vRefAbc = Math::singlePhaseVariableToThreePhase(vRef);

  **mX = Matrix::Zero(mNetworkStateSize, 1);
  for (Int p = 0; p < 3; ++p) {
    (**mX)(mVcRe[p], 0) = vcAbc(p, 0).real();
    (**mX)(mVcIm[p], 0) = vcAbc(p, 0).imag();
    (**mX)(mIfRe[p], 0) = ifAbc(p, 0).real();
    (**mX)(mIfIm[p], 0) = ifAbc(p, 0).imag();
  }
  **mIntfVoltage = u;
  **mIntfCurrent = (u - vcAbc) / mRc;

  const Real psi = std::arg(vc);
  const Complex rot = std::exp(-j * psi);
  const Complex vcDq = K32 * vc * rot;
  const Complex iDq = K32 * iGrid * rot;
  const Complex vRefDq = K32 * vRef * rot;
  const Real pInit = (vcDq * std::conj(iDq)).real();
  const Real qInit = (vcDq * std::conj(iDq)).imag();
  mControllerState.setZero();
  mControllerState(Psi, 0) = psi;
  mControllerState(PhiPLL, 0) = (omega - mOmegaN) / mKiPLL;
  mControllerState(PFiltered, 0) = pInit;
  mControllerState(QFiltered, 0) = qInit;
  mControllerState(PhiD, 0) =
      (iDq.real() + mKpPowerCtrl * (pInit - mPRef)) / mKiPowerCtrl;
  mControllerState(PhiQ, 0) =
      (iDq.imag() - mKpPowerCtrl * (qInit - mQRef)) / mKiPowerCtrl;
  const Real iRefD = -mKpPowerCtrl * pInit +
                     mKiPowerCtrl * mControllerState(PhiD, 0) +
                     mKpPowerCtrl * mPRef;
  const Real iRefQ = mKpPowerCtrl * qInit +
                     mKiPowerCtrl * mControllerState(PhiQ, 0) -
                     mKpPowerCtrl * mQRef;
  mControllerState(GammaD, 0) =
      (vRefDq.real() + mKpCurrCtrl * (iDq.real() - iRefD)) / mKiCurrCtrl;
  mControllerState(GammaQ, 0) =
      (vRefDq.imag() + mKpCurrCtrl * (iDq.imag() - iRefQ)) / mKiCurrCtrl;

  const Matrix uPacked = packComplex(u);
  mControllerMeasurementOld = buildControllerMeasurement(**mX, uPacked);
  mControllerOutput = packComplex(vRefAbc);
  mDelayedControllerOutput = mControllerOutput;
  buildControllerStateSpaceModel(mControllerState, mControllerMeasurementOld,
                                 mControllerA, mControllerB, mControllerC,
                                 mControllerD, mControllerE, mControllerF);
  if (mTimeStep > 0.0)
    recomputeDiscreteModel();
  updateLogAttributes(uPacked);
}

void DP::Ph3::SSN_GFL_Split::updateLogAttributes(const Matrix &u) const {
  const Matrix measurement = buildControllerMeasurement(**mX, u);
  Complex3 vc, iGrid;
  unpackMeasurement(measurement, vc, iGrid);
  const Complex3 projection = {Complex(1.0, 0.0), SHIFT_TO_PHASE_C,
                               SHIFT_TO_PHASE_B};
  Complex pV(0.0, 0.0), pI(0.0, 0.0);
  for (Int p = 0; p < 3; ++p) {
    pV += projection[p] * vc[p];
    pI += projection[p] * iGrid[p];
  }
  const Complex rot = std::exp(Complex(0.0, -mControllerState(Psi, 0)));
  const Complex vcDq = 0.5 * K23 * rot * pV;
  const Complex iDq = 0.5 * K23 * rot * pI;
  **mVcD = vcDq.real();
  **mVcQ = vcDq.imag();
  **mIrcD = iDq.real();
  **mIrcQ = iDq.imag();
  **mPInst = (vcDq * std::conj(iDq)).real();
  **mQInst = (vcDq * std::conj(iDq)).imag();
  **mOmegaPLL =
      mOmegaN + mKpPLL * **mVcQ + mKiPLL * mControllerState(PhiPLL, 0);
}

Matrix DP::Ph3::SSN_GFL_Split::getControllerState() const {
  return mControllerState;
}
Matrix DP::Ph3::SSN_GFL_Split::getNetworkState() const { return **mX; }
Matrix DP::Ph3::SSN_GFL_Split::getStateDerivative() const {
  Matrix controllerDerivative;
  const Matrix u = packComplex(**mIntfVoltage);
  const Matrix measurement = buildControllerMeasurement(**mX, u);
  evaluateControllerStateDerivative(mControllerState, measurement,
                                    controllerDerivative);
  Matrix result(20, 1);
  result.topRows(8) = controllerDerivative;
  result.bottomRows(12) =
      mA * (**mX) + mB * u + mBControllerOutput * mDelayedControllerOutput;
  return result;
}
Matrix DP::Ph3::SSN_GFL_Split::getInterfaceVoltagePacked() const {
  return packComplex(**mIntfVoltage);
}
Matrix DP::Ph3::SSN_GFL_Split::getInterfaceCurrentPacked() const {
  return packComplex(**mIntfCurrent);
}
Matrix DP::Ph3::SSN_GFL_Split::getConverterVoltageReference() const {
  return mControllerOutput;
}
Matrix DP::Ph3::SSN_GFL_Split::getDelayedConverterVoltage() const {
  return mDelayedControllerOutput;
}
Matrix DP::Ph3::SSN_GFL_Split::getControllerA() const { return mControllerA; }
Matrix DP::Ph3::SSN_GFL_Split::getControllerB() const { return mControllerB; }
Matrix DP::Ph3::SSN_GFL_Split::getControllerC() const { return mControllerC; }
Matrix DP::Ph3::SSN_GFL_Split::getControllerD() const { return mControllerD; }
Matrix DP::Ph3::SSN_GFL_Split::getNetworkA() const { return mA; }
Matrix DP::Ph3::SSN_GFL_Split::getNetworkB() const {
  Matrix result = Matrix::Zero(12, 12);
  result.leftCols(6) = mB;
  result.rightCols(6) = mBControllerOutput;
  return result;
}
Matrix DP::Ph3::SSN_GFL_Split::getNetworkC() const { return mC; }
Matrix DP::Ph3::SSN_GFL_Split::getNetworkD() const {
  Matrix result = Matrix::Zero(6, 12);
  result.leftCols(6) = mD;
  return result;
}
Matrix DP::Ph3::SSN_GFL_Split::getEquivalentConductance() const { return mW; }
