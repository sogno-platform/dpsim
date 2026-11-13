// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#include <dpsim-models/EMT/EMT_Ph3_SSN_SynchronGenerator4Order.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace CPS;
namespace {
constexpr Int kEd = 0, kEq = 1, kOmega = 2, kTheta = 3;
constexpr Real kSqrtTwoThirds = 0.8164965809277260327;
} // namespace

EMT::Ph3::SSN::SynchronGenerator4Order::SynchronGenerator4Order(
    String uid, String name, Logger::Level level)
    : TwoTerminalVTypeVariableSSNComp(uid, name, level) {
  **mX = Matrix::Zero(4, 1);
  setParameters(Matrix::Zero(4, 4), Matrix::Zero(4, 3), Matrix::Zero(3, 4),
                Matrix::Zero(3, 3));
}

void EMT::Ph3::SSN::SynchronGenerator4Order::setParametersOperationalPerUnit(
    Real nominalPower, Real nominalVoltage, Real nominalFrequency, Real inertia,
    Real resistance, Real xdTransient, Real xqTransient, Real xd, Real xq,
    Real td0Transient, Real tq0Transient, Real mechanicalDamping) {
  if (nominalPower <= 0. || nominalVoltage <= 0. || nominalFrequency <= 0. ||
      inertia <= 0. || td0Transient <= 0. || tq0Transient <= 0. ||
      xdTransient <= 0. || xqTransient <= 0.)
    throw std::invalid_argument("Invalid fourth-order SSN machine parameters.");
  mNominalPower = nominalPower;
  mNominalVoltage = nominalVoltage;
  mFrequency = nominalFrequency;
  mBaseV = nominalVoltage * kSqrtTwoThirds;
  mBaseI = 2. * nominalPower / (3. * mBaseV);
  mBaseZ = mBaseV / mBaseI;
  mOmega = 2. * PI * nominalFrequency;
  mH = inertia;
  mRa = resistance;
  mXd = xd;
  mXq = xq;
  mXdp = xdTransient;
  mXqp = xqTransient;
  mTd0p = td0Transient;
  mTq0p = tq0Transient;
  mDamping = mechanicalDamping;
  mConfigured = true;
}

void EMT::Ph3::SSN::SynchronGenerator4Order::setPowerflowInitialValues(
    Real activePowerPu, Real reactivePowerPu, Real voltageMagnitudePu,
    Real voltageAngleRad) {
  mPm = activePowerPu;
  mQm = reactivePowerPu;
  mVpu = voltageMagnitudePu;
  mVangle = voltageAngleRad;
  mInitialValuesConfigured = true;
}

void EMT::Ph3::SSN::SynchronGenerator4Order::evaluate(
    const Matrix &x, const Matrix &u, MatrixFixedSize<4, 1> &f,
    MatrixFixedSize<3, 1> &y) const {
  const Real theta = x(kTheta, 0);
  const Real phaseStep = 2. * PI / 3.;
  Real vd = 0., vq = 0.;
  Real cosPhase[3], sinPhase[3];
  for (Int p = 0; p < 3; ++p) {
    const Real angle = theta - phaseStep * p;
    cosPhase[p] = std::cos(angle);
    sinPhase[p] = std::sin(angle);
    vd += (2. / 3.) * cosPhase[p] * u(p, 0) / mBaseV;
    vq -= (2. / 3.) * sinPhase[p] * u(p, 0) / mBaseV;
  }
  const Real dd = vd - x(kEd, 0), dq = vq - x(kEq, 0);
  const Real det = mRa * mRa + mXdp * mXqp;
  const Real id = (mRa * dd + mXqp * dq) / det;
  const Real iq = (-mXdp * dd + mRa * dq) / det;
  for (Int p = 0; p < 3; ++p)
    y(p, 0) = mBaseI * (cosPhase[p] * id - sinPhase[p] * iq);

  f.setZero(4, 1);
  f(kEd, 0) = (-(x(kEd, 0) - mEd0) - (mXq - mXqp) * (iq - mIq0)) / mTq0p;
  f(kEq, 0) = (-(x(kEq, 0) - mEq0) + (mXd - mXdp) * (id - mId0)) / mTd0p;
  const Real pe = vd * id + vq * iq;
  f(kOmega, 0) = (mPm + pe - mDamping * (x(kOmega, 0) - 1.)) / (2. * mH);
  f(kTheta, 0) = mOmega * x(kOmega, 0);
}

void EMT::Ph3::SSN::SynchronGenerator4Order::linearize(const Matrix &x,
                                                       const Matrix &u,
                                                       Matrix &A, Matrix &B,
                                                       Matrix &C,
                                                       Matrix &D) const {
  // This model used to be linearized with 14 finite-difference evaluations of
  // evaluate() per timestep and generator.  Besides the repeated arithmetic,
  // that path rebuilt the Park transforms and several temporary Eigen
  // matrices for every perturbation.  The equations below are the exact
  // analytical Jacobian of evaluate(), avoiding that hot-path overhead.
  A.setZero(4, 4);
  B.setZero(4, 3);
  C.setZero(3, 4);
  D.setZero(3, 3);
  const Real theta = x(kTheta, 0);
  const Real thetaStep = 2. * PI / 3.;
  Real vd = 0., vq = 0.;
  Real dVdDu[3], dVqDu[3];
  Real cosPhase[3], sinPhase[3];
  for (Int p = 0; p < 3; ++p) {
    const Real angle = theta - thetaStep * p;
    cosPhase[p] = std::cos(angle);
    sinPhase[p] = std::sin(angle);
    vd += (2. / 3.) * cosPhase[p] * u(p, 0) / mBaseV;
    vq -= (2. / 3.) * sinPhase[p] * u(p, 0) / mBaseV;
    dVdDu[p] = (2. / 3.) * cosPhase[p] / mBaseV;
    dVqDu[p] = -(2. / 3.) * sinPhase[p] / mBaseV;
  }

  const Real det = mRa * mRa + mXdp * mXqp;
  const Real id = (mRa * (vd - x(kEd, 0)) + mXqp * (vq - x(kEq, 0))) / det;
  const Real iq = (-mXdp * (vd - x(kEd, 0)) + mRa * (vq - x(kEq, 0))) / det;
  const Real idVd = mRa / det;
  const Real idVq = mXqp / det;
  const Real iqVd = -mXdp / det;
  const Real iqVq = mRa / det;
  const Real idEd = -idVd, idEq = -idVq;
  const Real iqEd = -iqVd, iqEq = -iqVq;
  const Real vdTheta = vq;
  const Real vqTheta = -vd;
  const Real idTheta = idVd * vdTheta + idVq * vqTheta;
  const Real iqTheta = iqVd * vdTheta + iqVq * vqTheta;
  const Real edScale = -(mXq - mXqp) / mTq0p;
  const Real eqScale = (mXd - mXdp) / mTd0p;
  const Real powerScale = 1. / (2. * mH);

  A(0, 0) = -1. / mTq0p + edScale * iqEd;
  A(0, 1) = edScale * iqEq;
  A(0, 3) = edScale * iqTheta;
  A(1, 0) = eqScale * idEd;
  A(1, 1) = -1. / mTd0p + eqScale * idEq;
  A(1, 3) = eqScale * idTheta;
  A(2, 0) = powerScale * (vd * idEd + vq * iqEd);
  A(2, 1) = powerScale * (vd * idEq + vq * iqEq);
  A(2, 2) = -mDamping * powerScale;
  A(2, 3) =
      powerScale * (vdTheta * id + vd * idTheta + vqTheta * iq + vq * iqTheta);
  A(3, 2) = mOmega;

  for (Int p = 0; p < 3; ++p) {
    B(0, p) = edScale * (iqVd * dVdDu[p] + iqVq * dVqDu[p]);
    B(1, p) = eqScale * (idVd * dVdDu[p] + idVq * dVqDu[p]);
    B(2, p) = powerScale * (vd * (idVd * dVdDu[p] + idVq * dVqDu[p]) +
                            vq * (iqVd * dVdDu[p] + iqVq * dVqDu[p]));
  }

  for (Int p = 0; p < 3; ++p) {
    const Real rD = cosPhase[p];
    const Real rQ = -sinPhase[p];
    const Real drD = -sinPhase[p];
    const Real drQ = -cosPhase[p];
    C(p, 0) = mBaseI * (rD * idEd + rQ * iqEd);
    C(p, 1) = mBaseI * (rD * idEq + rQ * iqEq);
    C(p, 3) = mBaseI * (drD * id + drQ * iq + rD * idTheta + rQ * iqTheta);
    for (Int j = 0; j < 3; ++j) {
      const Real idU = idVd * dVdDu[j] + idVq * dVqDu[j];
      const Real iqU = iqVd * dVdDu[j] + iqVq * dVqDu[j];
      D(p, j) = mBaseI * (rD * idU + rQ * iqU);
    }
  }
}

Bool EMT::Ph3::SSN::SynchronGenerator4Order::updateComponentParameters() {
  linearize(**mX, **mIntfVoltage, mA, mB, mC, mD);
  evaluate(**mX, **mIntfVoltage, mEvaluationF, mEvaluationY);
  mStateOffset = mEvaluationF;
  mStateOffset.noalias() -= mA * (**mX);
  mStateOffset.noalias() -= mB * (**mIntfVoltage);
  mOutputOffset = mEvaluationY;
  mOutputOffset.noalias() -= mC * (**mX);
  mOutputOffset.noalias() -= mD * (**mIntfVoltage);
  setStateOffset(mStateOffset);
  setOutputOffset(mOutputOffset);
  return true;
}

void EMT::Ph3::SSN::SynchronGenerator4Order::initializeState(
    const Matrix &phaseVoltage) {
  if (!mConfigured || !mInitialValuesConfigured)
    throw std::logic_error(
        "Configure SSN generator parameters and PF values first.");
  const Complex v = mVpu * std::polar(1., mVangle);
  const Complex s(mPm, mQm);
  // Positive-sequence current is positive into the generator component.
  const Complex i = -std::conj(s / v);
  const Complex e = v - Complex(mRa, 0.5 * (mXdp + mXqp)) * i;
  const Real theta = std::arg(e) - PI / 2.;
  const Complex rot = std::polar(1., -theta);
  const Complex vdq = v * rot, idq = i * rot;
  (**mX)(kTheta, 0) = theta;
  (**mX)(kOmega, 0) = 1.;
  (**mX)(kEd, 0) = vdq.real() - mRa * idq.real() + mXqp * idq.imag();
  (**mX)(kEq, 0) = vdq.imag() - mXdp * idq.real() - mRa * idq.imag();
  mEd0 = (**mX)(kEd, 0);
  mEq0 = (**mX)(kEq, 0);
  mId0 = idq.real();
  mIq0 = idq.imag();
  (void)phaseVoltage;
}

void EMT::Ph3::SSN::SynchronGenerator4Order::initializeFromNodesAndTerminals(
    Real) {
  if (!mConfigured)
    throw std::logic_error("SSN generator parameters are unset.");
  MatrixComp ph = MatrixComp::Zero(3, 1);
  ph(0, 0) = kSqrtTwoThirds * initialSingleVoltage(1);
  ph(1, 0) = ph(0, 0) * SHIFT_TO_PHASE_B;
  ph(2, 0) = ph(0, 0) * SHIFT_TO_PHASE_C;
  const Complex solvedPower = -terminal(0)->singlePower();
  if (std::isfinite(solvedPower.real()) && std::isfinite(solvedPower.imag()) &&
      std::abs(solvedPower) > 1.) {
    mPm = solvedPower.real() / mNominalPower;
    mQm = solvedPower.imag() / mNominalPower;
  }
  mVpu = std::abs(ph(0, 0)) / mBaseV;
  mVangle = std::arg(ph(0, 0));
  **mIntfVoltage = ph.real();
  initializeState(**mIntfVoltage);
  updateComponentParameters();
  evaluate(**mX, **mIntfVoltage, mEvaluationF, mEvaluationY);
  **mIntfCurrent = mEvaluationY;
}

std::vector<String>
EMT::Ph3::SSN::SynchronGenerator4Order::getLocalStateNames() const {
  return {"EdTransient", "EqTransient", "speedPu", "rotorAngle"};
}
void EMT::Ph3::SSN::SynchronGenerator4Order::updateLogAttributes(
    const Matrix &) const {}
Matrix EMT::Ph3::SSN::SynchronGenerator4Order::getState() const { return **mX; }
