// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <stdexcept>

#include <dpsim-models/EMT/EMT_Ph3_SSN_GFL.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

EMT::Ph3::SSN_GFL::SSN_GFL(String uid, String name, Logger::Level logLevel)
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
  **mIntfVoltage = Matrix::Zero(mInputSize, 1);
  **mIntfCurrent = Matrix::Zero(mOutputSize, 1);

  **mVcD = 0.0;
  **mVcQ = 0.0;
  **mIrcD = 0.0;
  **mIrcQ = 0.0;
  **mPInst = 0.0;
  **mQInst = 0.0;
  **mOmegaPLL = 0.0;
}

std::vector<String> EMT::Ph3::SSN_GFL::getLocalStateNames() const {
  return {
      "theta_pll", "phi_pll", "p_filtered", "q_filtered", "phi_d",
      "phi_q",     "gamma_d", "gamma_q",    "vc_a",       "vc_b",
      "vc_c",      "if_a",    "if_b",       "if_c",
  };
}

std::vector<EMT::SSNComp::LocalAbcStateBlock>
EMT::Ph3::SSN_GFL::getLocalAbcStateBlocks() const {
  return {
      {{static_cast<UInt>(VcA), static_cast<UInt>(VcB), static_cast<UInt>(VcC)},
       "vc"},
      {{static_cast<UInt>(IfA), static_cast<UInt>(IfB), static_cast<UInt>(IfC)},
       "if"},
  };
}

void EMT::Ph3::SSN_GFL::setParameters(Real lf, Real cf, Real rf, Real rc,
                                      Real omegaN, Real kpPLL, Real kiPLL,
                                      Real omegaCutoff, Real pRef, Real qRef,
                                      Real kpPowerCtrl, Real kiPowerCtrl,
                                      Real kpCurrCtrl, Real kiCurrCtrl) {
  if (!Math::isFinite(lf) || !Math::isFinite(cf) || !Math::isFinite(rf) ||
      !Math::isFinite(rc) || !Math::isFinite(omegaN) ||
      !Math::isFinite(kpPLL) || !Math::isFinite(kiPLL) ||
      !Math::isFinite(omegaCutoff) || !Math::isFinite(pRef) ||
      !Math::isFinite(qRef) || !Math::isFinite(kpPowerCtrl) ||
      !Math::isFinite(kiPowerCtrl) || !Math::isFinite(kpCurrCtrl) ||
      !Math::isFinite(kiCurrCtrl))
    throw std::invalid_argument("SSN_GFL parameters must be finite.");

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
  const Matrix u0 = Matrix::Zero(mInputSize, 1);

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

MatrixFixedSize<2, 3>
EMT::Ph3::SSN_GFL::getParkTransformMatrix(Real theta) const {
  MatrixFixedSize<2, 3> transform;
  const Real scale = std::sqrt(2.0 / 3.0);

  transform.row(0) << scale * std::cos(theta),
      scale * std::cos(theta - 2.0 * PI / 3.0),
      scale * std::cos(theta + 2.0 * PI / 3.0);

  transform.row(1) << -scale * std::sin(theta),
      -scale * std::sin(theta - 2.0 * PI / 3.0),
      -scale * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

MatrixFixedSize<3, 2>
EMT::Ph3::SSN_GFL::getInverseParkTransformMatrix(Real theta) const {
  MatrixFixedSize<3, 2> transform;
  const Real scale = std::sqrt(2.0 / 3.0);

  transform << scale * std::cos(theta), -scale * std::sin(theta),
      scale * std::cos(theta - 2.0 * PI / 3.0),
      -scale * std::sin(theta - 2.0 * PI / 3.0),
      scale * std::cos(theta + 2.0 * PI / 3.0),
      -scale * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

void EMT::Ph3::SSN_GFL::evaluateStateDerivative(const Matrix &x,
                                                const Matrix &u,
                                                Matrix &stateDerivative) const {
  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL state vector has an invalid dimension.");

  if (u.rows() != mInputSize || u.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL input vector has an invalid dimension.");

  stateDerivative.setZero(mStateSize, 1);

  const Real thetaPLL = x(ThetaPLL, 0);
  const Real phiPLL = x(PhiPLL, 0);
  const Real pFiltered = x(PFiltered, 0);
  const Real qFiltered = x(QFiltered, 0);
  const Real phiD = x(PhiD, 0);
  const Real phiQ = x(PhiQ, 0);
  const Real gammaD = x(GammaD, 0);
  const Real gammaQ = x(GammaQ, 0);

  const MatrixFixedSize<3, 1> vcAbc = x.block(VcA, 0, 3, 1);
  const MatrixFixedSize<3, 1> ifAbc = x.block(IfA, 0, 3, 1);

  const MatrixFixedSize<2, 3> parkTransform = getParkTransformMatrix(thetaPLL);
  const MatrixFixedSize<3, 2> inverseParkTransform =
      getInverseParkTransformMatrix(thetaPLL);

  // Positive current is physical inverter injection into the grid.
  const MatrixFixedSize<3, 1> iGridAbc = (vcAbc - u) / mRc;

  const MatrixFixedSize<2, 1> vcDq = parkTransform * vcAbc;
  const MatrixFixedSize<2, 1> iGridDq = parkTransform * iGridAbc;

  const Real vcD = vcDq(0, 0);
  const Real vcQ = vcDq(1, 0);
  const Real iGridD = iGridDq(0, 0);
  const Real iGridQ = iGridDq(1, 0);

  // The power-invariant Park transformation requires no additional 3/2
  // scaling. These equations intentionally match the reference GFL model.
  const Real pInstantaneous = vcD * iGridD + vcQ * iGridQ;
  const Real qInstantaneous = -vcD * iGridQ + vcQ * iGridD;

  // ----------------------------------------------------------------------
  // 1. PLL
  // ----------------------------------------------------------------------
  stateDerivative(ThetaPLL, 0) = mOmegaN + mKpPLL * vcQ + mKiPLL * phiPLL;
  stateDerivative(PhiPLL, 0) = vcQ;

  // ----------------------------------------------------------------------
  // 2. Active- and reactive-power measurement filters
  // ----------------------------------------------------------------------
  stateDerivative(PFiltered, 0) = mOmegaCutoff * (pInstantaneous - pFiltered);
  stateDerivative(QFiltered, 0) = mOmegaCutoff * (qInstantaneous - qFiltered);

  // ----------------------------------------------------------------------
  // 3. Outer power controller
  // ----------------------------------------------------------------------
  stateDerivative(PhiD, 0) = mPRef - pFiltered;
  stateDerivative(PhiQ, 0) = qFiltered - mQRef;

  const Real currentReferenceD =
      mKpPowerCtrl * (mPRef - pFiltered) + mKiPowerCtrl * phiD;
  const Real currentReferenceQ =
      mKpPowerCtrl * (qFiltered - mQRef) + mKiPowerCtrl * phiQ;

  // ----------------------------------------------------------------------
  // 4. Inner current controller
  // ----------------------------------------------------------------------
  const Real currentErrorD = currentReferenceD - iGridD;
  const Real currentErrorQ = currentReferenceQ - iGridQ;

  stateDerivative(GammaD, 0) = currentErrorD;
  stateDerivative(GammaQ, 0) = currentErrorQ;

  const Real converterVoltageReferenceD =
      mKpCurrCtrl * currentErrorD + mKiCurrCtrl * gammaD;
  const Real converterVoltageReferenceQ =
      mKpCurrCtrl * currentErrorQ + mKiCurrCtrl * gammaQ;

  MatrixFixedSize<2, 1> converterVoltageReferenceDq;
  converterVoltageReferenceDq << converterVoltageReferenceD,
      converterVoltageReferenceQ;

  const MatrixFixedSize<3, 1> converterVoltageReferenceAbc =
      inverseParkTransform * converterVoltageReferenceDq;

  // ----------------------------------------------------------------------
  // 5. Electrical filter plant
  // ----------------------------------------------------------------------
  const MatrixFixedSize<3, 1> vcDerivative =
      ifAbc / mCf + (u - vcAbc) / (mCf * mRc);
  const MatrixFixedSize<3, 1> ifDerivative =
      (converterVoltageReferenceAbc - vcAbc - mRf * ifAbc) / mLf;

  stateDerivative.block(VcA, 0, 3, 1) = vcDerivative;
  stateDerivative.block(IfA, 0, 3, 1) = ifDerivative;
}

void EMT::Ph3::SSN_GFL::evaluateOutput(const Matrix &x, const Matrix &u,
                                       Matrix &output) const {
  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL state vector has an invalid dimension.");

  if (u.rows() != mInputSize || u.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL input vector has an invalid dimension.");

  const Matrix vcAbc = x.block(VcA, 0, 3, 1);

  // SSN current entering the component. The controller uses the opposite
  // physical current direction as inverter injection into the grid.
  output = (u - vcAbc) / mRc;
}

void EMT::Ph3::SSN_GFL::calculateAnalyticalJacobians(const Matrix &x,
                                                     const Matrix &u, Matrix &A,
                                                     Matrix &B, Matrix &C,
                                                     Matrix &D) const {
  if (x.rows() != mStateSize || x.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL state vector has an invalid dimension.");

  if (u.rows() != mInputSize || u.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL input vector has an invalid dimension.");

  const Real thetaPLL = x(ThetaPLL, 0);
  const Real pFiltered = x(PFiltered, 0);
  const Real qFiltered = x(QFiltered, 0);
  const Real phiD = x(PhiD, 0);
  const Real phiQ = x(PhiQ, 0);
  const Real gammaD = x(GammaD, 0);
  const Real gammaQ = x(GammaQ, 0);
  const MatrixFixedSize<3, 1> vcAbc = x.block(VcA, 0, 3, 1);

  const MatrixFixedSize<3, 3> identity3 = Matrix::Identity(3, 3);

  const MatrixFixedSize<2, 3> parkTransform = getParkTransformMatrix(thetaPLL);
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> tD = parkTransform.row(0);
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> tQ = parkTransform.row(1);

  const MatrixFixedSize<3, 2> inverseParkTransform =
      getInverseParkTransformMatrix(thetaPLL);
  const MatrixFixedSize<3, 1> sD = inverseParkTransform.col(0);
  const MatrixFixedSize<3, 1> sQ = inverseParkTransform.col(1);

  // Exact derivatives of the orthonormal Park transformation used by the
  // reference model.
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dTdTheta = tQ;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dTqTheta = -tD;
  const MatrixFixedSize<3, 1> dSdTheta = sQ;
  const MatrixFixedSize<3, 1> dSqTheta = -sD;

  const MatrixFixedSize<3, 1> iGridAbc = (vcAbc - u) / mRc;

  const Real vcD = (tD * vcAbc)(0, 0);
  const Real vcQ = (tQ * vcAbc)(0, 0);
  const Real iGridD = (tD * iGridAbc)(0, 0);
  const Real iGridQ = (tQ * iGridAbc)(0, 0);

  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dVcDByVc = tD;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dVcQByVc = tQ;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dIGridDByVc = tD / mRc;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dIGridQByVc = tQ / mRc;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dIGridDByU = -tD / mRc;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dIGridQByU = -tQ / mRc;

  const Real dVcDByTheta = (dTdTheta * vcAbc)(0, 0);
  const Real dVcQByTheta = (dTqTheta * vcAbc)(0, 0);
  const Real dIGridDByTheta = (dTdTheta * iGridAbc)(0, 0);
  const Real dIGridQByTheta = (dTqTheta * iGridAbc)(0, 0);

  // Instantaneous active-power Jacobian.
  const Real dPByTheta = iGridD * dVcDByTheta + vcD * dIGridDByTheta +
                         iGridQ * dVcQByTheta + vcQ * dIGridQByTheta;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dPByVc =
      iGridD * dVcDByVc + vcD * dIGridDByVc + iGridQ * dVcQByVc +
      vcQ * dIGridQByVc;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dPByU =
      vcD * dIGridDByU + vcQ * dIGridQByU;

  // Instantaneous reactive-power Jacobian.
  const Real dQByTheta = -iGridQ * dVcDByTheta - vcD * dIGridQByTheta +
                         iGridD * dVcQByTheta + vcQ * dIGridDByTheta;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dQByVc =
      -iGridQ * dVcDByVc - vcD * dIGridQByVc + iGridD * dVcQByVc +
      vcQ * dIGridDByVc;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dQByU =
      -vcD * dIGridQByU + vcQ * dIGridDByU;

  const Real currentReferenceD =
      -mKpPowerCtrl * pFiltered + mKiPowerCtrl * phiD + mKpPowerCtrl * mPRef;
  const Real currentReferenceQ =
      mKpPowerCtrl * qFiltered + mKiPowerCtrl * phiQ - mKpPowerCtrl * mQRef;

  const Real converterVoltageReferenceD = -mKpCurrCtrl * iGridD +
                                          mKiCurrCtrl * gammaD +
                                          mKpCurrCtrl * currentReferenceD;
  const Real converterVoltageReferenceQ = -mKpCurrCtrl * iGridQ +
                                          mKiCurrCtrl * gammaQ +
                                          mKpCurrCtrl * currentReferenceQ;

  const Real dVoltageReferenceDByTheta = -mKpCurrCtrl * dIGridDByTheta;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dVoltageReferenceDByVc =
      -mKpCurrCtrl * dIGridDByVc;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dVoltageReferenceDByU =
      -mKpCurrCtrl * dIGridDByU;

  const Real dVoltageReferenceQByTheta = -mKpCurrCtrl * dIGridQByTheta;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dVoltageReferenceQByVc =
      -mKpCurrCtrl * dIGridQByVc;
  const Eigen::Matrix<Real, 1, 3, Eigen::RowMajor> dVoltageReferenceQByU =
      -mKpCurrCtrl * dIGridQByU;

  MatrixFixedSize<3, 14> dConverterVoltageAbcByX =
      MatrixFixedSize<3, 14>::Zero();
  MatrixFixedSize<3, 3> dConverterVoltageAbcByU = MatrixFixedSize<3, 3>::Zero();

  dConverterVoltageAbcByX.col(ThetaPLL) =
      dSdTheta * converterVoltageReferenceD +
      dSqTheta * converterVoltageReferenceQ + sD * dVoltageReferenceDByTheta +
      sQ * dVoltageReferenceQByTheta;

  dConverterVoltageAbcByX.col(PFiltered) += sD * (-mKpCurrCtrl * mKpPowerCtrl);
  dConverterVoltageAbcByX.col(PhiD) += sD * (mKpCurrCtrl * mKiPowerCtrl);
  dConverterVoltageAbcByX.col(GammaD) += sD * mKiCurrCtrl;

  dConverterVoltageAbcByX.col(QFiltered) += sQ * (mKpCurrCtrl * mKpPowerCtrl);
  dConverterVoltageAbcByX.col(PhiQ) += sQ * (mKpCurrCtrl * mKiPowerCtrl);
  dConverterVoltageAbcByX.col(GammaQ) += sQ * mKiCurrCtrl;

  dConverterVoltageAbcByX.block(0, VcA, 3, 3) +=
      sD * dVoltageReferenceDByVc + sQ * dVoltageReferenceQByVc;

  dConverterVoltageAbcByU =
      sD * dVoltageReferenceDByU + sQ * dVoltageReferenceQByU;

  A.setZero(mStateSize, mStateSize);
  B.setZero(mStateSize, mInputSize);
  C.setZero(mOutputSize, mStateSize);
  D.setZero(mOutputSize, mInputSize);

  // PLL rows.
  A(ThetaPLL, ThetaPLL) = mKpPLL * dVcQByTheta;
  A(ThetaPLL, PhiPLL) = mKiPLL;
  A.block(ThetaPLL, VcA, 1, 3) = mKpPLL * dVcQByVc;

  A(PhiPLL, ThetaPLL) = dVcQByTheta;
  A.block(PhiPLL, VcA, 1, 3) = dVcQByVc;

  // Power-filter rows.
  A(PFiltered, ThetaPLL) = mOmegaCutoff * dPByTheta;
  A(PFiltered, PFiltered) = -mOmegaCutoff;
  A.block(PFiltered, VcA, 1, 3) = mOmegaCutoff * dPByVc;
  B.block(PFiltered, 0, 1, 3) = mOmegaCutoff * dPByU;

  A(QFiltered, ThetaPLL) = mOmegaCutoff * dQByTheta;
  A(QFiltered, QFiltered) = -mOmegaCutoff;
  A.block(QFiltered, VcA, 1, 3) = mOmegaCutoff * dQByVc;
  B.block(QFiltered, 0, 1, 3) = mOmegaCutoff * dQByU;

  // Outer power-loop integrators.
  A(PhiD, PFiltered) = -1.0;
  A(PhiQ, QFiltered) = 1.0;

  // Inner current-loop integrators.
  A(GammaD, PFiltered) = -mKpPowerCtrl;
  A(GammaD, PhiD) = mKiPowerCtrl;
  A(GammaD, ThetaPLL) = -dIGridDByTheta;
  A.block(GammaD, VcA, 1, 3) = -dIGridDByVc;
  B.block(GammaD, 0, 1, 3) = -dIGridDByU;

  A(GammaQ, QFiltered) = mKpPowerCtrl;
  A(GammaQ, PhiQ) = mKiPowerCtrl;
  A(GammaQ, ThetaPLL) = -dIGridQByTheta;
  A.block(GammaQ, VcA, 1, 3) = -dIGridQByVc;
  B.block(GammaQ, 0, 1, 3) = -dIGridQByU;

  // Electrical filter plant.
  A.block(VcA, VcA, 3, 3) = -1.0 / (mCf * mRc) * identity3;
  A.block(VcA, IfA, 3, 3) = 1.0 / mCf * identity3;
  B.block(VcA, 0, 3, 3) = 1.0 / (mCf * mRc) * identity3;

  A.block(IfA, 0, 3, mStateSize) = (1.0 / mLf) * dConverterVoltageAbcByX;
  A.block(IfA, VcA, 3, 3) += -1.0 / mLf * identity3;
  A.block(IfA, IfA, 3, 3) += -mRf / mLf * identity3;
  B.block(IfA, 0, 3, 3) = (1.0 / mLf) * dConverterVoltageAbcByU;

  // SSN output y = (u - vc) / Rc.
  C.block(0, VcA, 3, 3) = -1.0 / mRc * identity3;
  D = 1.0 / mRc * identity3;
}

void EMT::Ph3::SSN_GFL::buildStateSpaceModel(const Matrix &x, const Matrix &u,
                                             Matrix &A, Matrix &B, Matrix &C,
                                             Matrix &D, Matrix &E, Matrix &F) {
  calculateAnalyticalJacobians(x, u, A, B, C, D);

  evaluateStateDerivative(x, u, mStateDerivativeScratch);
  evaluateOutput(x, u, mOutputScratch);

  // Local affine offsets at the operating point.
  E = mStateDerivativeScratch;
  E.noalias() -= A * x;
  E.noalias() -= B * u;
  F = mOutputScratch;
  F.noalias() -= C * x;
  F.noalias() -= D * u;
}

Bool EMT::Ph3::SSN_GFL::updateComponentParameters() {
  buildStateSpaceModel(**mX, **mIntfVoltage, mA, mB, mC, mD,
                       mStateOffsetScratch, mOutputOffsetScratch);

  setStateOffset(mStateOffsetScratch);
  setOutputOffset(mOutputOffsetScratch);

  // The PLL and dq/abc transformations make the local affine state-space
  // model time varying. The SSN equivalent is therefore rebuilt every step.
  return true;
}

void EMT::Ph3::SSN_GFL::updateLogAttributes(const Matrix &u) const {
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

void EMT::Ph3::SSN_GFL::initializeFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  // Initialize EMT abc filter states and dq-frame controller states.
  const Real omega = 2.0 * PI * frequency;
  const Complex imaginaryUnit(0.0, 1.0);
  const Complex powerReference(mPRef, mQRef);

  const MatrixComp uPhasor = buildInitialInputFromNodes(frequency);

  MatrixComp vcPhasor = uPhasor;
  MatrixComp injectionCurrentPhasor = MatrixComp::Zero(3, 1);

  for (Int iteration = 0; iteration < mInitializationMaxIterations;
       ++iteration) {
    const Complex vcA = vcPhasor(0, 0);

    if (std::abs(vcA) < mInitializationTolerance) {
      injectionCurrentPhasor.setZero();
      break;
    }

    // Peak-valued phase phasors use S = 1.5*V_peak*conj(I_peak).
    const Complex currentA = std::conj(powerReference / (1.5 * vcA));

    MatrixComp nextInjectionCurrent(3, 1);
    nextInjectionCurrent << currentA, currentA * SHIFT_TO_PHASE_B,
        currentA * SHIFT_TO_PHASE_C;

    const MatrixComp nextVcPhasor = uPhasor + mRc * nextInjectionCurrent;

    injectionCurrentPhasor = nextInjectionCurrent;

    if ((nextVcPhasor - vcPhasor).norm() < mInitializationTolerance) {
      vcPhasor = nextVcPhasor;
      break;
    }

    vcPhasor = nextVcPhasor;
  }

  const MatrixComp ifPhasor =
      imaginaryUnit * omega * mCf * vcPhasor + injectionCurrentPhasor;
  const MatrixComp converterVoltageReferencePhasor =
      vcPhasor + (mRf + imaginaryUnit * omega * mLf) * ifPhasor;

  const Matrix vcAbc0 = vcPhasor.real();
  const Matrix ifAbc0 = ifPhasor.real();
  const Matrix injectionCurrentAbc0 = injectionCurrentPhasor.real();
  const Matrix converterVoltageReferenceAbc0 =
      converterVoltageReferencePhasor.real();

  const Real theta0 = std::arg(vcPhasor(0, 0));
  const Matrix parkTransform = getParkTransformMatrix(theta0);

  const Matrix vcDq0 = parkTransform * vcAbc0;
  const Matrix injectionCurrentDq0 = parkTransform * injectionCurrentAbc0;
  const Matrix converterVoltageReferenceDq0 =
      parkTransform * converterVoltageReferenceAbc0;

  const Real vcD0 = vcDq0(0, 0);
  const Real vcQ0 = vcDq0(1, 0);
  const Real iGridD0 = injectionCurrentDq0(0, 0);
  const Real iGridQ0 = injectionCurrentDq0(1, 0);

  const Real pInitial = vcD0 * iGridD0 + vcQ0 * iGridQ0;
  const Real qInitial = -vcD0 * iGridQ0 + vcQ0 * iGridD0;

  Matrix x0 = Matrix::Zero(mStateSize, 1);

  x0(ThetaPLL, 0) = theta0;
  x0(PhiPLL, 0) = (omega - mOmegaN) / mKiPLL;
  x0(PFiltered, 0) = pInitial;
  x0(QFiltered, 0) = qInitial;

  x0(PhiD, 0) = (iGridD0 + mKpPowerCtrl * (pInitial - mPRef)) / mKiPowerCtrl;
  x0(PhiQ, 0) = (iGridQ0 - mKpPowerCtrl * (qInitial - mQRef)) / mKiPowerCtrl;

  const Real currentReferenceD = -mKpPowerCtrl * pInitial +
                                 mKiPowerCtrl * x0(PhiD, 0) +
                                 mKpPowerCtrl * mPRef;
  const Real currentReferenceQ = mKpPowerCtrl * qInitial +
                                 mKiPowerCtrl * x0(PhiQ, 0) -
                                 mKpPowerCtrl * mQRef;

  x0(GammaD, 0) = (converterVoltageReferenceDq0(0, 0) +
                   mKpCurrCtrl * (iGridD0 - currentReferenceD)) /
                  mKiCurrCtrl;
  x0(GammaQ, 0) = (converterVoltageReferenceDq0(1, 0) +
                   mKpCurrCtrl * (iGridQ0 - currentReferenceQ)) /
                  mKiCurrCtrl;

  x0.block(VcA, 0, 3, 1) = vcAbc0;
  x0.block(IfA, 0, 3, 1) = ifAbc0;

  **mX = x0;
  **mIntfVoltage = uPhasor.real();
  **mIntfCurrent = ((uPhasor - vcPhasor) / mRc).real();

  // Update local continuous-time matrices at the initialized operating point.
  updateComponentParameters();
  updateLogAttributes(**mIntfVoltage);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- SSN GFL phasor/dq initialization ---"
                     "\nInput u: {:s}"
                     "\nOutput y: {:s}"
                     "\nState x: {:s}"
                     "\nP/Q init: [{:.6e}, {:.6e}]"
                     "\nVc dq: [{:.6e}, {:.6e}]"
                     "\nIgrid dq: [{:.6e}, {:.6e}]"
                     "\n--- SSN GFL initialization finished ---",
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent),
                     Logger::matrixToString(**mX), pInitial, qInitial, vcD0,
                     vcQ0, iGridD0, iGridQ0);
}

Matrix EMT::Ph3::SSN_GFL::getState() const { return **mX; }

Matrix EMT::Ph3::SSN_GFL::getStateDerivative() const {
  Matrix stateDerivative = Matrix::Zero(mStateSize, 1);
  evaluateStateDerivative(**mX, **mIntfVoltage, stateDerivative);
  return stateDerivative;
}

Matrix EMT::Ph3::SSN_GFL::getInterfaceVoltage() const { return **mIntfVoltage; }

Matrix EMT::Ph3::SSN_GFL::getInterfaceCurrent() const { return **mIntfCurrent; }
