// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0
#include <dpsim-models/DP/DP_Ph3_SSN_SynchronGenerator4Order.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

using namespace CPS;
namespace {
constexpr Int kEd = 0, kEq = 1, kOmega = 2, kTheta = 3;
const Complex alpha = std::polar(1., 2. * PI / 3.);
MatrixComp unpack3(const Matrix &u) {
  MatrixComp v(3, 1);
  for (Int p = 0; p < 3; ++p)
    v(p, 0) = Complex(u(2 * p, 0), u(2 * p + 1, 0));
  return v;
}
Matrix pack3(const MatrixComp &y) {
  Matrix out(6, 1);
  for (Int p = 0; p < 3; ++p) {
    out(2 * p, 0) = y(p, 0).real();
    out(2 * p + 1, 0) = y(p, 0).imag();
  }
  return out;
}
} // namespace

DP::Ph3::SSN::SynchronGenerator4Order::SynchronGenerator4Order(
    String uid, String name, Logger::Level level)
    : MixedVTypeVariableSSNComp(uid, name, 4, 0, level) {
  **mX = Matrix::Zero(4, 1);
  setParameters(Matrix::Zero(4, 4), Matrix::Zero(4, 6), Matrix::Zero(6, 4),
                Matrix::Zero(6, 6));
}

void DP::Ph3::SSN::SynchronGenerator4Order::setParametersOperationalPerUnit(
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
  mBaseV = nominalVoltage * 0.8164965809277260327;
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

void DP::Ph3::SSN::SynchronGenerator4Order::setPowerflowInitialValues(
    Real activePowerPu, Real reactivePowerPu, Real voltageMagnitudePu,
    Real voltageAngleRad) {
  mPm = activePowerPu;
  mQm = reactivePowerPu;
  mVpu = voltageMagnitudePu;
  mVangle = voltageAngleRad;
  mInitialValuesConfigured = true;
}

void DP::Ph3::SSN::SynchronGenerator4Order::evaluate(const Matrix &x,
                                                     const Matrix &u, Matrix &f,
                                                     Matrix &y) const {
  f.setZero(4, 1);
  y.resize(6, 1);
  const Complex v =
      (Complex(u(0, 0), u(1, 0)) + alpha * Complex(u(2, 0), u(3, 0)) +
       std::conj(alpha) * Complex(u(4, 0), u(5, 0))) /
      (3. * mBaseV);
  const Complex vdq = v * std::polar(1., -x(kTheta, 0));
  const Real vd = vdq.real(), vq = vdq.imag();
  const Real dd = vd - x(kEd, 0), dq = vq - x(kEq, 0),
             det = mRa * mRa + mXdp * mXqp;
  const Real id = (mRa * dd + mXqp * dq) / det,
             iq = (-mXdp * dd + mRa * dq) / det;
  const Complex idq(id, iq), iph = mBaseI * idq * std::polar(1., x(kTheta, 0));
  const std::array<Complex, 3> currents = {iph, iph * std::conj(alpha),
                                           iph * alpha};
  for (Int phase = 0; phase < 3; ++phase) {
    y(2 * phase, 0) = currents[phase].real();
    y(2 * phase + 1, 0) = currents[phase].imag();
  }
  f(kEd, 0) = (-(x(kEd, 0) - mEd0) - (mXq - mXqp) * (iq - mIq0)) / mTq0p;
  f(kEq, 0) = (-(x(kEq, 0) - mEq0) + (mXd - mXdp) * (id - mId0)) / mTd0p;
  const Real pe = vd * id + vq * iq;
  f(kOmega, 0) = (mPm + pe - mDamping * (x(kOmega, 0) - 1.)) / (2. * mH);
  f(kTheta, 0) = mOmega * (x(kOmega, 0) - 1.);
}

void DP::Ph3::SSN::SynchronGenerator4Order::linearize(const Matrix &x,
                                                      const Matrix &u,
                                                      Matrix &A, Matrix &B,
                                                      Matrix &C,
                                                      Matrix &D) const {
  A.setZero(4, 4);
  B.setZero(4, 6);
  C.setZero(6, 4);
  D.setZero(6, 6);

  // Positive-sequence projection and its exact derivatives with respect to
  // the packed real/imaginary phase-envelope inputs.
  const std::array<Complex, 3> sequenceCoeff = {Complex(1., 0.), alpha,
                                                std::conj(alpha)};
  Complex positiveSequence = 0.;
  for (Int phase = 0; phase < 3; ++phase)
    positiveSequence +=
        sequenceCoeff[phase] * Complex(u(2 * phase, 0), u(2 * phase + 1, 0));
  positiveSequence /= 3. * mBaseV;

  const Real theta = x(kTheta, 0);
  const Complex dqVoltage = positiveSequence * std::polar(1., -theta);
  const Real vd = dqVoltage.real();
  const Real vq = dqVoltage.imag();

  std::array<Real, 6> dVdDu{};
  std::array<Real, 6> dVqDu{};
  const Complex rotateToDQ = std::polar(1., -theta) / (3. * mBaseV);
  for (Int phase = 0; phase < 3; ++phase) {
    const Complex dVdRe = sequenceCoeff[phase] * rotateToDQ;
    const Complex dVdIm = Complex(0., 1.) * dVdRe;
    dVdDu[2 * phase] = dVdRe.real();
    dVqDu[2 * phase] = dVdRe.imag();
    dVdDu[2 * phase + 1] = dVdIm.real();
    dVqDu[2 * phase + 1] = dVdIm.imag();
  }

  const Real det = mRa * mRa + mXdp * mXqp;
  const Real idVd = mRa / det;
  const Real idVq = mXqp / det;
  const Real iqVd = -mXdp / det;
  const Real iqVq = mRa / det;
  const Real idEd = -idVd;
  const Real idEq = -idVq;
  const Real iqEd = -iqVd;
  const Real iqEq = -iqVq;

  const Real dd = vd - x(kEd, 0);
  const Real dq = vq - x(kEq, 0);
  const Real id = (mRa * dd + mXqp * dq) / det;
  const Real iq = (-mXdp * dd + mRa * dq) / det;

  // d(vd,vq)/d(theta) = (vq,-vd).
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

  for (Int input = 0; input < 6; ++input) {
    const Real idDu = idVd * dVdDu[input] + idVq * dVqDu[input];
    const Real iqDu = iqVd * dVdDu[input] + iqVq * dVqDu[input];
    B(0, input) = edScale * iqDu;
    B(1, input) = eqScale * idDu;
    B(2, input) = powerScale * (vd * idDu + vq * iqDu);
  }

  const Complex idq(id, iq);
  const Complex phaseCurrentGain = mBaseI * std::polar(1., theta);
  const std::array<Complex, 3> phaseCoeff = {Complex(1., 0.), std::conj(alpha),
                                             alpha};
  for (Int phase = 0; phase < 3; ++phase) {
    const Complex gain = phaseCurrentGain * phaseCoeff[phase];
    const Int reRow = 2 * phase;
    const Int imRow = reRow + 1;

    const Complex dIdqDEd(idEd, iqEd);
    const Complex dIdqDEq(idEq, iqEq);
    const Complex dIdqDTheta(idTheta, iqTheta);
    const Complex dITheta = gain * (Complex(0., 1.) * idq + dIdqDTheta);
    const Complex dIEd = gain * dIdqDEd;
    const Complex dIEq = gain * dIdqDEq;
    C(reRow, 0) = dIEd.real();
    C(imRow, 0) = dIEd.imag();
    C(reRow, 1) = dIEq.real();
    C(imRow, 1) = dIEq.imag();
    C(reRow, 3) = dITheta.real();
    C(imRow, 3) = dITheta.imag();

    for (Int input = 0; input < 6; ++input) {
      const Real idDu = idVd * dVdDu[input] + idVq * dVqDu[input];
      const Real iqDu = iqVd * dVdDu[input] + iqVq * dVqDu[input];
      const Complex dI = gain * Complex(idDu, iqDu);
      D(reRow, input) = dI.real();
      D(imRow, input) = dI.imag();
    }
  }
}

Bool DP::Ph3::SSN::SynchronGenerator4Order::updateComponentParameters() {
  for (Int phase = 0; phase < 3; ++phase) {
    const Complex voltage = (**mIntfVoltage)(phase, 0);
    mPackedVoltage(2 * phase, 0) = voltage.real();
    mPackedVoltage(2 * phase + 1, 0) = voltage.imag();
  }

  linearize(**mX, mPackedVoltage, mA, mB, mC, mD);
  evaluate(**mX, mPackedVoltage, mEvaluationDerivative, mEvaluationOutput);

  mE = mEvaluationDerivative;
  mE.noalias() -= mA * (**mX);
  mE.noalias() -= mB * mPackedVoltage;
  mF = mEvaluationOutput;
  mF.noalias() -= mC * (**mX);
  mF.noalias() -= mD * mPackedVoltage;
  return true;
}

void DP::Ph3::SSN::SynchronGenerator4Order::initializeState(
    const MatrixComp &phaseVoltage) {
  if (!mConfigured || !mInitialValuesConfigured)
    throw std::logic_error(
        "Configure SSN generator parameters and PF values first.");
  const Complex v = mVpu * std::polar(1., mVangle);
  const Complex i = -std::conj(Complex(mPm, mQm) / v);
  const Complex e = v - Complex(mRa, .5 * (mXdp + mXqp)) * i;
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

void DP::Ph3::SSN::SynchronGenerator4Order::initializeFromNodesAndTerminals(
    Real) {
  if (!mConfigured)
    throw std::logic_error("SSN generator parameters are unset.");
  MatrixComp voltage = MatrixComp::Zero(3, 1);
  voltage(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1);
  voltage(1, 0) = voltage(0, 0) * SHIFT_TO_PHASE_B;
  voltage(2, 0) = voltage(0, 0) * SHIFT_TO_PHASE_C;
  // SystemTopology transfers solved SG power onto terminal 0 for the matching
  // power-flow component name, including the slack-bus active-power result.
  const Complex solvedPower = -terminal(0)->singlePower();
  if (std::isfinite(solvedPower.real()) && std::isfinite(solvedPower.imag()) &&
      std::abs(solvedPower) > 1.) {
    mPm = solvedPower.real() / mNominalPower;
    mQm = solvedPower.imag() / mNominalPower;
  }
  mVpu = std::abs(voltage(0, 0)) / mBaseV;
  mVangle = std::arg(voltage(0, 0));
  **mIntfVoltage = voltage;
  initializeState(voltage);
  updateComponentParameters();
  Matrix f, y;
  evaluate(**mX, pack3(voltage), f, y);
  **mIntfCurrent = unpack3(y);
}

void DP::Ph3::SSN::SynchronGenerator4Order::updateLogAttributes(
    const Matrix &) const {}
Matrix DP::Ph3::SSN::SynchronGenerator4Order::getState() const { return **mX; }
