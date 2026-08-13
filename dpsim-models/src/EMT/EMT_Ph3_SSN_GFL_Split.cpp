// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <stdexcept>

#include <dpsim-models/EMT/EMT_Ph3_SSN_GFL_Split.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

EMT::Ph3::SSN_GFL_Split::SSN_GFL_Split(String uid, String name,
                                       Logger::Level logLevel)
    : TwoTerminalVTypeSSNComp(uid, name, logLevel), mLf(0.0), mCf(0.0),
      mRf(0.0), mRc(0.0), mOmegaN(0.0), mKpPLL(0.0), mKiPLL(0.0),
      mOmegaCutoff(0.0), mPRef(0.0), mQRef(0.0), mKpPowerCtrl(0.0),
      mKiPowerCtrl(0.0), mKpCurrCtrl(0.0), mKiCurrCtrl(0.0),
      mBConverter(Matrix::Zero(mNetworkStateSize, 3)),
      mdBConverter(Matrix::Zero(mNetworkStateSize, 3)),
      mControllerState(Matrix::Zero(mControllerStateSize, 1)),
      mControllerMeasurementOld(Matrix::Zero(mControllerInputSize, 1)),
      mControllerA(Matrix::Zero(mControllerStateSize, mControllerStateSize)),
      mControllerB(Matrix::Zero(mControllerStateSize, mControllerInputSize)),
      mControllerC(Matrix::Zero(mControllerOutputSize, mControllerStateSize)),
      mControllerD(Matrix::Zero(mControllerOutputSize, mControllerInputSize)),
      mControllerE(Matrix::Zero(mControllerStateSize, 1)),
      mControllerF(Matrix::Zero(mControllerOutputSize, 1)),
      mControllerAd(Matrix::Zero(mControllerStateSize, mControllerStateSize)),
      mControllerBd(Matrix::Zero(mControllerStateSize, mControllerInputSize)),
      mControllerEd(Matrix::Zero(mControllerStateSize, 1)),
      mConverterVoltageReference(Matrix::Zero(3, 1)),
      mConverterVoltageDelayed(Matrix::Zero(3, 1)),
      mVcD(mAttributes->create<Real>("vc_d")),
      mVcQ(mAttributes->create<Real>("vc_q")),
      mIrcD(mAttributes->create<Real>("irc_d")),
      mIrcQ(mAttributes->create<Real>("irc_q")),
      mPInst(mAttributes->create<Real>("p_inst")),
      mQInst(mAttributes->create<Real>("q_inst")),
      mOmegaPLL(mAttributes->create<Real>("omega_pll")),
      mVInvRefA(mAttributes->create<Real>("vinv_ref_a")),
      mVInvRefB(mAttributes->create<Real>("vinv_ref_b")),
      mVInvRefC(mAttributes->create<Real>("vinv_ref_c")) {
  **mIntfVoltage = Matrix::Zero(mTerminalInputSize, 1);
  **mIntfCurrent = Matrix::Zero(mOutputSize, 1);

  **mVcD = 0.0;
  **mVcQ = 0.0;
  **mIrcD = 0.0;
  **mIrcQ = 0.0;
  **mPInst = 0.0;
  **mQInst = 0.0;
  **mOmegaPLL = 0.0;

  **mVInvRefA = 0.0;
  **mVInvRefB = 0.0;
  **mVInvRefC = 0.0;
}

std::vector<String> EMT::Ph3::SSN_GFL_Split::getLocalStateNames() const {
  // Only the fixed electrical plant belongs to the SSN state vector.
  return {"vc_a", "vc_b", "vc_c", "if_a", "if_b", "if_c"};
}

std::vector<EMT::SSNComp::LocalAbcStateBlock>
EMT::Ph3::SSN_GFL_Split::getLocalAbcStateBlocks() const {
  return {
      {{static_cast<UInt>(VcA), static_cast<UInt>(VcB), static_cast<UInt>(VcC)},
       "vc"},
      {{static_cast<UInt>(IfA), static_cast<UInt>(IfB), static_cast<UInt>(IfC)},
       "if"},
  };
}

void EMT::Ph3::SSN_GFL_Split::setParameters(Real lf, Real cf, Real rf, Real rc,
                                            Real omegaN, Real kpPLL, Real kiPLL,
                                            Real omegaCutoff, Real pRef,
                                            Real qRef, Real kpPowerCtrl,
                                            Real kiPowerCtrl, Real kpCurrCtrl,
                                            Real kiCurrCtrl) {

  if (!Math::isFinite(lf) || !Math::isFinite(cf) || !Math::isFinite(rf) ||
      !Math::isFinite(rc) || !Math::isFinite(omegaN) ||
      !Math::isFinite(kpPLL) || !Math::isFinite(kiPLL) ||
      !Math::isFinite(omegaCutoff) || !Math::isFinite(pRef) ||
      !Math::isFinite(qRef) || !Math::isFinite(kpPowerCtrl) ||
      !Math::isFinite(kiPowerCtrl) || !Math::isFinite(kpCurrCtrl) ||
      !Math::isFinite(kiCurrCtrl))
    throw std::invalid_argument("SSN_GFL_Split parameters must be finite.");

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

  const Matrix identity3 = Matrix::Identity(3, 3);

  // Constant electrical plant.
  Matrix aNetwork = Matrix::Zero(mNetworkStateSize, mNetworkStateSize);
  Matrix bTerminal = Matrix::Zero(mNetworkStateSize, mTerminalInputSize);
  Matrix cNetwork = Matrix::Zero(mOutputSize, mNetworkStateSize);
  Matrix dTerminal = Matrix::Zero(mOutputSize, mTerminalInputSize);

  aNetwork.block(VcA, VcA, 3, 3) = -1.0 / (mCf * mRc) * identity3;
  aNetwork.block(VcA, IfA, 3, 3) = 1.0 / mCf * identity3;

  aNetwork.block(IfA, VcA, 3, 3) = -1.0 / mLf * identity3;
  aNetwork.block(IfA, IfA, 3, 3) = -mRf / mLf * identity3;

  bTerminal.block(VcA, 0, 3, 3) = 1.0 / (mCf * mRc) * identity3;

  mBConverter.setZero(mNetworkStateSize, 3);
  mBConverter.block(IfA, 0, 3, 3) = 1.0 / mLf * identity3;

  cNetwork.block(0, VcA, 3, 3) = -1.0 / mRc * identity3;
  dTerminal = 1.0 / mRc * identity3;

  // This fixed SSN plant is not changed after setParameters().
  SSNComp::setParameters(aNetwork, bTerminal, cNetwork, dTerminal);

  mControllerState.setZero();
  mControllerMeasurementOld.setZero();
  mConverterVoltageReference.setZero();
  mConverterVoltageDelayed.setZero();

  // Initialize controller matrix dimensions and a valid operating-point model.
  buildControllerStateSpaceModel(mControllerState, mControllerMeasurementOld,
                                 mControllerA, mControllerB, mControllerC,
                                 mControllerD, mControllerE, mControllerF);
}

Matrix EMT::Ph3::SSN_GFL_Split::getParkTransformMatrix(Real theta) const {
  Matrix transform(2, 3);
  const Real scale = std::sqrt(2.0 / 3.0);

  transform.row(0) << scale * std::cos(theta),
      scale * std::cos(theta - 2.0 * PI / 3.0),
      scale * std::cos(theta + 2.0 * PI / 3.0);

  transform.row(1) << -scale * std::sin(theta),
      -scale * std::sin(theta - 2.0 * PI / 3.0),
      -scale * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

Matrix
EMT::Ph3::SSN_GFL_Split::getInverseParkTransformMatrix(Real theta) const {
  Matrix transform(3, 2);
  const Real scale = std::sqrt(2.0 / 3.0);

  transform << scale * std::cos(theta), -scale * std::sin(theta),
      scale * std::cos(theta - 2.0 * PI / 3.0),
      -scale * std::sin(theta - 2.0 * PI / 3.0),
      scale * std::cos(theta + 2.0 * PI / 3.0),
      -scale * std::sin(theta + 2.0 * PI / 3.0);

  return transform;
}

Matrix EMT::Ph3::SSN_GFL_Split::buildControllerMeasurement(
    const Matrix &networkState, const Matrix &terminalVoltage) const {

  if (networkState.rows() != mNetworkStateSize || networkState.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split network state has an invalid dimension.");

  if (terminalVoltage.rows() != mTerminalInputSize ||
      terminalVoltage.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split terminal voltage has an invalid dimension.");

  const Matrix vcAbc = networkState.block(VcA, 0, 3, 1);

  // Positive current is physical inverter injection into the grid.
  const Matrix iGridAbc = (vcAbc - terminalVoltage) / mRc;

  Matrix measurement = Matrix::Zero(mControllerInputSize, 1);
  measurement.block(0, 0, 3, 1) = vcAbc;
  measurement.block(3, 0, 3, 1) = iGridAbc;

  return measurement;
}

void EMT::Ph3::SSN_GFL_Split::evaluateControllerStateDerivative(
    const Matrix &xController, const Matrix &measurement,
    Matrix &stateDerivative) const {

  if (xController.rows() != mControllerStateSize || xController.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split controller state has an invalid dimension.");

  if (measurement.rows() != mControllerInputSize || measurement.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split controller measurement has an invalid dimension.");

  stateDerivative.setZero(mControllerStateSize, 1);

  const Real thetaPLL = xController(ThetaPLL, 0);
  const Real phiPLL = xController(PhiPLL, 0);
  const Real pFiltered = xController(PFiltered, 0);
  const Real qFiltered = xController(QFiltered, 0);
  const Real phiD = xController(PhiD, 0);
  const Real phiQ = xController(PhiQ, 0);

  const Matrix vcAbc = measurement.block(0, 0, 3, 1);
  const Matrix iGridAbc = measurement.block(3, 0, 3, 1);

  const Matrix parkTransform = getParkTransformMatrix(thetaPLL);

  const Matrix vcDq = parkTransform * vcAbc;
  const Matrix iGridDq = parkTransform * iGridAbc;

  const Real vcD = vcDq(0, 0);
  const Real vcQ = vcDq(1, 0);
  const Real iGridD = iGridDq(0, 0);
  const Real iGridQ = iGridDq(1, 0);

  const Real pInstantaneous = vcD * iGridD + vcQ * iGridQ;
  const Real qInstantaneous = -vcD * iGridQ + vcQ * iGridD;

  // 1. PLL
  stateDerivative(ThetaPLL, 0) = mOmegaN + mKpPLL * vcQ + mKiPLL * phiPLL;
  stateDerivative(PhiPLL, 0) = vcQ;

  // 2. Active-/reactive-power filters
  stateDerivative(PFiltered, 0) = mOmegaCutoff * (pInstantaneous - pFiltered);
  stateDerivative(QFiltered, 0) = mOmegaCutoff * (qInstantaneous - qFiltered);

  // 3. Outer power loop
  stateDerivative(PhiD, 0) = mPRef - pFiltered;
  stateDerivative(PhiQ, 0) = qFiltered - mQRef;

  const Real currentReferenceD =
      mKpPowerCtrl * (mPRef - pFiltered) + mKiPowerCtrl * phiD;
  const Real currentReferenceQ =
      mKpPowerCtrl * (qFiltered - mQRef) + mKiPowerCtrl * phiQ;

  // 4. Inner current-loop integrators
  stateDerivative(GammaD, 0) = currentReferenceD - iGridD;
  stateDerivative(GammaQ, 0) = currentReferenceQ - iGridQ;
}

void EMT::Ph3::SSN_GFL_Split::evaluateControllerOutput(
    const Matrix &xController, const Matrix &measurement,
    Matrix &output) const {

  if (xController.rows() != mControllerStateSize || xController.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split controller state has an invalid dimension.");

  if (measurement.rows() != mControllerInputSize || measurement.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split controller measurement has an invalid dimension.");

  const Real thetaPLL = xController(ThetaPLL, 0);
  const Real pFiltered = xController(PFiltered, 0);
  const Real qFiltered = xController(QFiltered, 0);
  const Real phiD = xController(PhiD, 0);
  const Real phiQ = xController(PhiQ, 0);
  const Real gammaD = xController(GammaD, 0);
  const Real gammaQ = xController(GammaQ, 0);

  const Matrix iGridAbc = measurement.block(3, 0, 3, 1);

  const Matrix parkTransform = getParkTransformMatrix(thetaPLL);
  const Matrix inverseParkTransform = getInverseParkTransformMatrix(thetaPLL);

  const Matrix iGridDq = parkTransform * iGridAbc;
  const Real iGridD = iGridDq(0, 0);
  const Real iGridQ = iGridDq(1, 0);

  const Real currentReferenceD =
      mKpPowerCtrl * (mPRef - pFiltered) + mKiPowerCtrl * phiD;
  const Real currentReferenceQ =
      mKpPowerCtrl * (qFiltered - mQRef) + mKiPowerCtrl * phiQ;

  const Real currentErrorD = currentReferenceD - iGridD;
  const Real currentErrorQ = currentReferenceQ - iGridQ;

  const Real converterVoltageReferenceD =
      mKpCurrCtrl * currentErrorD + mKiCurrCtrl * gammaD;
  const Real converterVoltageReferenceQ =
      mKpCurrCtrl * currentErrorQ + mKiCurrCtrl * gammaQ;

  Matrix converterVoltageReferenceDq(2, 1);
  converterVoltageReferenceDq << converterVoltageReferenceD,
      converterVoltageReferenceQ;

  output = inverseParkTransform * converterVoltageReferenceDq;
}

void EMT::Ph3::SSN_GFL_Split::calculateControllerAnalyticalJacobians(
    const Matrix &xController, const Matrix &measurement, Matrix &A, Matrix &B,
    Matrix &C, Matrix &D) const {

  if (xController.rows() != mControllerStateSize || xController.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split controller state has an invalid dimension.");

  if (measurement.rows() != mControllerInputSize || measurement.cols() != 1)
    throw std::invalid_argument(
        "SSN_GFL_Split controller measurement has an invalid dimension.");

  const Real thetaPLL = xController(ThetaPLL, 0);
  const Real pFiltered = xController(PFiltered, 0);
  const Real qFiltered = xController(QFiltered, 0);
  const Real phiD = xController(PhiD, 0);
  const Real phiQ = xController(PhiQ, 0);
  const Real gammaD = xController(GammaD, 0);
  const Real gammaQ = xController(GammaQ, 0);

  const Matrix vcAbc = measurement.block(0, 0, 3, 1);
  const Matrix iGridAbc = measurement.block(3, 0, 3, 1);

  const Matrix parkTransform = getParkTransformMatrix(thetaPLL);
  const Matrix tD = parkTransform.row(0);
  const Matrix tQ = parkTransform.row(1);

  const Matrix inverseParkTransform = getInverseParkTransformMatrix(thetaPLL);
  const Matrix sD = inverseParkTransform.col(0);
  const Matrix sQ = inverseParkTransform.col(1);

  // Exact derivatives of the orthonormal Park transformation.
  const Matrix dTdTheta = tQ;
  const Matrix dTqTheta = -tD;
  const Matrix dSdTheta = sQ;
  const Matrix dSqTheta = -sD;

  const Real vcD = (tD * vcAbc)(0, 0);
  const Real vcQ = (tQ * vcAbc)(0, 0);
  const Real iGridD = (tD * iGridAbc)(0, 0);
  const Real iGridQ = (tQ * iGridAbc)(0, 0);

  const Real dVcDByTheta = (dTdTheta * vcAbc)(0, 0);
  const Real dVcQByTheta = (dTqTheta * vcAbc)(0, 0);
  const Real dIGridDByTheta = (dTdTheta * iGridAbc)(0, 0);
  const Real dIGridQByTheta = (dTqTheta * iGridAbc)(0, 0);

  const Matrix dPByVc = iGridD * tD + iGridQ * tQ;
  const Matrix dPByI = vcD * tD + vcQ * tQ;

  const Real dPByTheta = iGridD * dVcDByTheta + vcD * dIGridDByTheta +
                         iGridQ * dVcQByTheta + vcQ * dIGridQByTheta;

  const Matrix dQByVc = -iGridQ * tD + iGridD * tQ;
  const Matrix dQByI = -vcD * tQ + vcQ * tD;

  const Real dQByTheta = -iGridQ * dVcDByTheta - vcD * dIGridQByTheta +
                         iGridD * dVcQByTheta + vcQ * dIGridDByTheta;

  const Real currentReferenceD =
      mKpPowerCtrl * (mPRef - pFiltered) + mKiPowerCtrl * phiD;
  const Real currentReferenceQ =
      mKpPowerCtrl * (qFiltered - mQRef) + mKiPowerCtrl * phiQ;

  const Real currentErrorD = currentReferenceD - iGridD;
  const Real currentErrorQ = currentReferenceQ - iGridQ;

  const Real converterVoltageReferenceD =
      mKpCurrCtrl * currentErrorD + mKiCurrCtrl * gammaD;
  const Real converterVoltageReferenceQ =
      mKpCurrCtrl * currentErrorQ + mKiCurrCtrl * gammaQ;

  A.setZero(mControllerStateSize, mControllerStateSize);
  B.setZero(mControllerStateSize, mControllerInputSize);
  C.setZero(mControllerOutputSize, mControllerStateSize);
  D.setZero(mControllerOutputSize, mControllerInputSize);

  // -----------------------------------------------------------------------
  // Controller state Jacobian A_c and measurement Jacobian B_c
  // -----------------------------------------------------------------------

  // PLL
  A(ThetaPLL, ThetaPLL) = mKpPLL * dVcQByTheta;
  A(ThetaPLL, PhiPLL) = mKiPLL;
  B.block(ThetaPLL, 0, 1, 3) = mKpPLL * tQ;

  A(PhiPLL, ThetaPLL) = dVcQByTheta;
  B.block(PhiPLL, 0, 1, 3) = tQ;

  // Power filters
  A(PFiltered, ThetaPLL) = mOmegaCutoff * dPByTheta;
  A(PFiltered, PFiltered) = -mOmegaCutoff;
  B.block(PFiltered, 0, 1, 3) = mOmegaCutoff * dPByVc;
  B.block(PFiltered, 3, 1, 3) = mOmegaCutoff * dPByI;

  A(QFiltered, ThetaPLL) = mOmegaCutoff * dQByTheta;
  A(QFiltered, QFiltered) = -mOmegaCutoff;
  B.block(QFiltered, 0, 1, 3) = mOmegaCutoff * dQByVc;
  B.block(QFiltered, 3, 1, 3) = mOmegaCutoff * dQByI;

  // Outer-loop integrators
  A(PhiD, PFiltered) = -1.0;
  A(PhiQ, QFiltered) = 1.0;

  // Inner-loop integrators
  A(GammaD, ThetaPLL) = -dIGridDByTheta;
  A(GammaD, PFiltered) = -mKpPowerCtrl;
  A(GammaD, PhiD) = mKiPowerCtrl;
  B.block(GammaD, 3, 1, 3) = -tD;

  A(GammaQ, ThetaPLL) = -dIGridQByTheta;
  A(GammaQ, QFiltered) = mKpPowerCtrl;
  A(GammaQ, PhiQ) = mKiPowerCtrl;
  B.block(GammaQ, 3, 1, 3) = -tQ;

  // -----------------------------------------------------------------------
  // Controller output Jacobian C_c and D_c
  //
  // v_inv,abc = S(theta) [v_ref,d; v_ref,q]
  // -----------------------------------------------------------------------
  C.col(ThetaPLL) = dSdTheta * converterVoltageReferenceD +
                    dSqTheta * converterVoltageReferenceQ +
                    sD * (-mKpCurrCtrl * dIGridDByTheta) +
                    sQ * (-mKpCurrCtrl * dIGridQByTheta);

  C.col(PFiltered) += sD * (-mKpCurrCtrl * mKpPowerCtrl);
  C.col(PhiD) += sD * (mKpCurrCtrl * mKiPowerCtrl);
  C.col(GammaD) += sD * mKiCurrCtrl;

  C.col(QFiltered) += sQ * (mKpCurrCtrl * mKpPowerCtrl);
  C.col(PhiQ) += sQ * (mKpCurrCtrl * mKiPowerCtrl);
  C.col(GammaQ) += sQ * mKiCurrCtrl;

  D.block(0, 3, 3, 3) = sD * (-mKpCurrCtrl * tD) + sQ * (-mKpCurrCtrl * tQ);
}

void EMT::Ph3::SSN_GFL_Split::buildControllerStateSpaceModel(
    const Matrix &xController, const Matrix &measurement, Matrix &A, Matrix &B,
    Matrix &C, Matrix &D, Matrix &E, Matrix &F) const {

  calculateControllerAnalyticalJacobians(xController, measurement, A, B, C, D);

  Matrix stateDerivative = Matrix::Zero(mControllerStateSize, 1);
  Matrix output = Matrix::Zero(mControllerOutputSize, 1);

  evaluateControllerStateDerivative(xController, measurement, stateDerivative);
  evaluateControllerOutput(xController, measurement, output);

  E = stateDerivative - A * xController - B * measurement;
  F = output - C * xController - D * measurement;
}

void EMT::Ph3::SSN_GFL_Split::recomputeControllerDiscreteModel() {
  if (mTimeStep <= 0.0)
    return;

  Math::calculateStateSpaceTrapezoidalMatrices(
      mControllerA, mControllerB, mControllerE, mTimeStep, mControllerAd,
      mControllerBd, mControllerEd);
}

void EMT::Ph3::SSN_GFL_Split::recomputeDiscreteModel() {
  // Fixed electrical plant.
  SSNComp::recomputeDiscreteModel();

  // Additional trapezoidal input matrix for the delayed converter command.
  const Matrix identity =
      Matrix::Identity(mNetworkStateSize, mNetworkStateSize);
  const Matrix lhs = identity - 0.5 * mTimeStep * mA;

  mdBConverter = lhs.fullPivLu().solve(0.5 * mTimeStep * mBConverter);

  // The controller is the only time-varying local subsystem.
  recomputeControllerDiscreteModel();
}

Matrix EMT::Ph3::SSN_GFL_Split::calculateHistoryVector() const {
  // The delayed converter voltage only enters the history vector.
  return mC * (mdA * (**mX) + mdB * (**mIntfVoltage) +
               2.0 * mdBConverter * mConverterVoltageDelayed);
}

void EMT::Ph3::SSN_GFL_Split::updateState(const Matrix &uOld,
                                          const Matrix &uNew) {
  // Use the converter command available before this network solve.
  const Matrix delayedVoltageUsedThisStep = mConverterVoltageDelayed;

  **mX = mdA * (**mX) + mdB * (uNew + uOld) +
         2.0 * mdBConverter * delayedVoltageUsedThisStep;

  // Form new measurements from the solved network state.
  const Matrix measurementNew = buildControllerMeasurement(**mX, uNew);

  // Advance the controller with the previous local affine model.
  mControllerState =
      mControllerAd * mControllerState +
      mControllerBd * (measurementNew + mControllerMeasurementOld) +
      mControllerEd;

  mControllerMeasurementOld = measurementNew;

  // Re-linearize only the controller at the new operating point.
  buildControllerStateSpaceModel(mControllerState, measurementNew, mControllerA,
                                 mControllerB, mControllerC, mControllerD,
                                 mControllerE, mControllerF);

  recomputeControllerDiscreteModel();

  // Calculate controller output at sample k.
  evaluateControllerOutput(mControllerState, measurementNew,
                           mConverterVoltageReference);

  // Store the command for the next simulation interval.
  mConverterVoltageDelayed = mConverterVoltageReference;
}

void EMT::Ph3::SSN_GFL_Split::updateLogAttributes(const Matrix &u) const {
  const Matrix measurement = buildControllerMeasurement(**mX, u);

  const Real thetaPLL = mControllerState(ThetaPLL, 0);
  const Matrix parkTransform = getParkTransformMatrix(thetaPLL);

  const Matrix vcAbc = measurement.block(0, 0, 3, 1);
  const Matrix iGridAbc = measurement.block(3, 0, 3, 1);

  **mVcD = (parkTransform.row(0) * vcAbc)(0, 0);
  **mVcQ = (parkTransform.row(1) * vcAbc)(0, 0);
  **mIrcD = (parkTransform.row(0) * iGridAbc)(0, 0);
  **mIrcQ = (parkTransform.row(1) * iGridAbc)(0, 0);

  **mPInst = **mVcD * **mIrcD + **mVcQ * **mIrcQ;
  **mQInst = -**mVcD * **mIrcQ + **mVcQ * **mIrcD;

  **mOmegaPLL =
      mOmegaN + mKpPLL * **mVcQ + mKiPLL * mControllerState(PhiPLL, 0);

  **mVInvRefA = mConverterVoltageReference(0, 0);
  **mVInvRefB = mConverterVoltageReference(1, 0);
  **mVInvRefC = mConverterVoltageReference(2, 0);
}

void EMT::Ph3::SSN_GFL_Split::initializeFromNodesAndTerminals(Real frequency) {

  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  // Initialize electrical and controller states at the same operating point.
  const Real omega = 2.0 * PI * frequency;
  const Complex imaginaryUnit(0.0, 1.0);
  const Complex powerReference(mPRef, mQRef);

  const MatrixComp uPhasor = buildInitialInputFromNodes(frequency);

  MatrixComp vcPhasor = uPhasor;
  MatrixComp injectionCurrentPhasor = MatrixComp::Zero(3, 1);

  constexpr Int initializationMaxIterations = 10;
  constexpr Real initializationTolerance = 1e-9;

  for (Int iteration = 0; iteration < initializationMaxIterations;
       ++iteration) {
    const Complex vcA = vcPhasor(0, 0);

    if (std::abs(vcA) < initializationTolerance) {
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

    if ((nextVcPhasor - vcPhasor).norm() < initializationTolerance) {
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

  // -----------------------------------------------------------------------
  // Network state
  // -----------------------------------------------------------------------
  Matrix xNetwork0 = Matrix::Zero(mNetworkStateSize, 1);
  xNetwork0.block(VcA, 0, 3, 1) = vcAbc0;
  xNetwork0.block(IfA, 0, 3, 1) = ifAbc0;

  **mX = xNetwork0;

  // -----------------------------------------------------------------------
  // Controller state
  // -----------------------------------------------------------------------
  mControllerState.setZero();

  mControllerState(ThetaPLL, 0) = theta0;
  mControllerState(PhiPLL, 0) = (omega - mOmegaN) / mKiPLL;
  mControllerState(PFiltered, 0) = pInitial;
  mControllerState(QFiltered, 0) = qInitial;

  mControllerState(PhiD, 0) =
      (iGridD0 + mKpPowerCtrl * (pInitial - mPRef)) / mKiPowerCtrl;

  mControllerState(PhiQ, 0) =
      (iGridQ0 - mKpPowerCtrl * (qInitial - mQRef)) / mKiPowerCtrl;

  const Real currentReferenceD = -mKpPowerCtrl * pInitial +
                                 mKiPowerCtrl * mControllerState(PhiD, 0) +
                                 mKpPowerCtrl * mPRef;

  const Real currentReferenceQ = mKpPowerCtrl * qInitial +
                                 mKiPowerCtrl * mControllerState(PhiQ, 0) -
                                 mKpPowerCtrl * mQRef;

  mControllerState(GammaD, 0) = (converterVoltageReferenceDq0(0, 0) +
                                 mKpCurrCtrl * (iGridD0 - currentReferenceD)) /
                                mKiCurrCtrl;

  mControllerState(GammaQ, 0) = (converterVoltageReferenceDq0(1, 0) +
                                 mKpCurrCtrl * (iGridQ0 - currentReferenceQ)) /
                                mKiCurrCtrl;

  // -----------------------------------------------------------------------
  // Interface quantities and controller measurement
  // -----------------------------------------------------------------------
  **mIntfVoltage = uPhasor.real();
  **mIntfCurrent = ((uPhasor - vcPhasor) / mRc).real();

  mControllerMeasurementOld = buildControllerMeasurement(**mX, **mIntfVoltage);

  // Avoid an artificial one-step zero command at t = 0.
  mConverterVoltageReference = converterVoltageReferenceAbc0;
  mConverterVoltageDelayed = converterVoltageReferenceAbc0;

  // Build the controller model at the initialized operating point.
  buildControllerStateSpaceModel(mControllerState, mControllerMeasurementOld,
                                 mControllerA, mControllerB, mControllerC,
                                 mControllerD, mControllerE, mControllerF);

  // Usually mTimeStep is set later by mnaCompInitialize(), which will call
  // recomputeDiscreteModel(). This also makes re-initialization robust if a
  // timestep is already known.
  if (mTimeStep > 0.0)
    recomputeControllerDiscreteModel();

  updateLogAttributes(**mIntfVoltage);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- SSN GFL split initialization ---"
      "\nInput u: {:s}"
      "\nOutput y: {:s}"
      "\nNetwork state x_n: {:s}"
      "\nController state x_c: {:s}"
      "\nP/Q init: [{:.6e}, {:.6e}]"
      "\nVc dq: [{:.6e}, {:.6e}]"
      "\nIgrid dq: [{:.6e}, {:.6e}]"
      "\nInitial delayed v_inv: {:s}"
      "\n--- SSN GFL split initialization finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent), Logger::matrixToString(**mX),
      Logger::matrixToString(mControllerState), pInitial, qInitial, vcD0, vcQ0,
      iGridD0, iGridQ0, Logger::matrixToString(mConverterVoltageDelayed));
}

Matrix EMT::Ph3::SSN_GFL_Split::getState() const {
  Matrix state = Matrix::Zero(mControllerStateSize + mNetworkStateSize, 1);

  state.block(0, 0, mControllerStateSize, 1) = mControllerState;
  state.block(mControllerStateSize, 0, mNetworkStateSize, 1) = **mX;

  return state;
}

Matrix EMT::Ph3::SSN_GFL_Split::getControllerState() const {
  return mControllerState;
}

Matrix EMT::Ph3::SSN_GFL_Split::getNetworkState() const { return **mX; }

Matrix EMT::Ph3::SSN_GFL_Split::getStateDerivative() const {
  const Matrix measurement = buildControllerMeasurement(**mX, **mIntfVoltage);

  Matrix controllerDerivative = Matrix::Zero(mControllerStateSize, 1);
  evaluateControllerStateDerivative(mControllerState, measurement,
                                    controllerDerivative);

  const Matrix networkDerivative = mA * (**mX) + mB * (**mIntfVoltage) +
                                   mBConverter * mConverterVoltageDelayed;

  Matrix derivative = Matrix::Zero(mControllerStateSize + mNetworkStateSize, 1);

  derivative.block(0, 0, mControllerStateSize, 1) = controllerDerivative;
  derivative.block(mControllerStateSize, 0, mNetworkStateSize, 1) =
      networkDerivative;

  return derivative;
}

Matrix EMT::Ph3::SSN_GFL_Split::getInterfaceVoltage() const {
  return **mIntfVoltage;
}

Matrix EMT::Ph3::SSN_GFL_Split::getInterfaceCurrent() const {
  return **mIntfCurrent;
}

Matrix EMT::Ph3::SSN_GFL_Split::getConverterVoltageReference() const {
  return mConverterVoltageReference;
}

Matrix EMT::Ph3::SSN_GFL_Split::getDelayedConverterVoltage() const {
  return mConverterVoltageDelayed;
}

Matrix EMT::Ph3::SSN_GFL_Split::getControllerA() const { return mControllerA; }

Matrix EMT::Ph3::SSN_GFL_Split::getControllerB() const { return mControllerB; }

Matrix EMT::Ph3::SSN_GFL_Split::getControllerC() const { return mControllerC; }

Matrix EMT::Ph3::SSN_GFL_Split::getControllerD() const { return mControllerD; }

Matrix EMT::Ph3::SSN_GFL_Split::getControllerE() const { return mControllerE; }

Matrix EMT::Ph3::SSN_GFL_Split::getControllerF() const { return mControllerF; }

Matrix EMT::Ph3::SSN_GFL_Split::getNetworkA() const { return mA; }

Matrix EMT::Ph3::SSN_GFL_Split::getNetworkB() const {
  Matrix bFull = Matrix::Zero(mNetworkStateSize, 6);

  bFull.block(0, 0, mNetworkStateSize, 3) = mB;
  bFull.block(0, 3, mNetworkStateSize, 3) = mBConverter;

  return bFull;
}

Matrix EMT::Ph3::SSN_GFL_Split::getNetworkC() const { return mC; }

Matrix EMT::Ph3::SSN_GFL_Split::getNetworkD() const {
  Matrix dFull = Matrix::Zero(mOutputSize, 6);

  dFull.block(0, 0, mOutputSize, 3) = mD;

  return dFull;
}

Matrix EMT::Ph3::SSN_GFL_Split::getEquivalentConductance() const { return mW; }
