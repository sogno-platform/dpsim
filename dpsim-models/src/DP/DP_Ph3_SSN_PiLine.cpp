/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph3_SSN_PiLine.h>
using namespace CPS;

namespace {
constexpr Int kSeriesCurrent = 0;
constexpr Int kSeriesInductorVoltage = 3;
constexpr Int kCapVoltage0 = 6;
constexpr Int kCapCurrent0 = 9;
constexpr Int kCapVoltage1 = 12;
constexpr Int kCapCurrent1 = 15;
} // namespace

DP::Ph3::SSN::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : MultiPortVTypeSSNComp(uid, name, logLevel),
      Base::Ph3::PiLine(mAttributes),
      mInductorAdmittance(MatrixComp::Zero(3, 3)),
      mSeriesAdmittance(MatrixComp::Zero(3, 3)),
      mSeriesSystemInverse(MatrixComp::Identity(3, 3)),
      mCapAdmittance(MatrixComp::Zero(3, 3)),
      mCapPreviousVoltage(MatrixComp::Zero(3, 3)),
      mSeriesHistory(MatrixComp::Zero(3, 1)),
      mInductorHistory(MatrixComp::Zero(3, 1)),
      mCapHistory0(MatrixComp::Zero(3, 1)),
      mCapHistory1(MatrixComp::Zero(3, 1)) {
  **mX = MatrixComp::Zero(18, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::SSN::PiLine::clone(String copySuffix) {
  auto copy = SharedFactory<PiLine>::make(copySuffix, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void DP::Ph3::SSN::PiLine::setParameters(const Matrix &seriesResistance,
                                         const Matrix &seriesInductance,
                                         const Matrix &parallelCapacitance,
                                         const Matrix &parallelConductance) {
  for (const Matrix *parameter : {&seriesResistance, &seriesInductance,
                                  &parallelCapacitance, &parallelConductance}) {
    if (parameter->rows() != 3 || parameter->cols() != 3)
      throw std::invalid_argument("PiLine parameter matrices must be 3 x 3.");
  }
  Base::Ph3::PiLine::setParameters(seriesResistance, seriesInductance,
                                   parallelCapacitance, parallelConductance);
  DP::SSNComp::setParameters(Matrix::Zero(18, 18), Matrix::Zero(18, 6),
                             Matrix::Zero(6, 18), Matrix::Zero(6, 6));
}

void DP::Ph3::SSN::PiLine::initializeFromNodesAndTerminals(Real frequency) {
  if ((**mParallelCond)(0, 0) <= 0.) {
    **mParallelCond = Matrix::Zero(3, 3);
    (**mParallelCond).diagonal().setConstant(1e-6);
  }

  const Real omega = 2. * PI * frequency;
  MatrixComp v0 = MatrixComp::Zero(3, 1);
  MatrixComp v1 = MatrixComp::Zero(3, 1);
  v0(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  v1(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1);
  v0(1, 0) = v0(0, 0) * SHIFT_TO_PHASE_B;
  v0(2, 0) = v0(0, 0) * SHIFT_TO_PHASE_C;
  v1(1, 0) = v1(0, 0) * SHIFT_TO_PHASE_B;
  v1(2, 0) = v1(0, 0) * SHIFT_TO_PHASE_C;

  const MatrixComp voltage = v1 - v0;
  MatrixComp impedance = (**mSeriesRes).cast<Complex>() +
                         Complex(0., omega) * (**mSeriesInd).cast<Complex>();
  const MatrixComp current = impedance.inverse() * voltage;
  const MatrixComp inductorVoltage = voltage - **mSeriesRes * current;
  const MatrixComp cap0Current =
      Complex(0., omega) * ((**mParallelCap) / 2.).cast<Complex>() * v0;
  const MatrixComp cap1Current =
      Complex(0., omega) * ((**mParallelCap) / 2.).cast<Complex>() * v1;

  **mIntfVoltage = MatrixComp::Zero(6, 1);
  (**mIntfVoltage).block(0, 0, 3, 1) = v0;
  (**mIntfVoltage).block(3, 0, 3, 1) = v1;
  **mIntfCurrent = MatrixComp::Zero(6, 1);
  (**mIntfCurrent).block(0, 0, 3, 1) = -current + cap0Current;
  (**mIntfCurrent).block(3, 0, 3, 1) = current + cap1Current;
  (**mX).setZero();
  (**mX).block<3, 1>(kSeriesCurrent, 0) = current;
  (**mX).block<3, 1>(kSeriesInductorVoltage, 0) = inductorVoltage;
  (**mX).block<3, 1>(kCapVoltage0, 0) = v0;
  (**mX).block<3, 1>(kCapCurrent0, 0) = cap0Current;
  (**mX).block<3, 1>(kCapVoltage1, 0) = v1;
  (**mX).block<3, 1>(kCapCurrent1, 0) = cap1Current;
}

void DP::Ph3::SSN::PiLine::recomputeDiscreteModel(Real omega) {
  const Real timeStep = mTimeStep;
  const Real b = timeStep * omega / 2.;
  const Matrix inverseL = (**mSeriesInd).inverse();
  const MatrixComp a = (timeStep / 2.) * inverseL.cast<Complex>();
  mInductorAdmittance = a / Complex(1., b);
  mInductorPreviousCurrent =
      Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
  mSeriesSystemInverse = (MatrixComp::Identity(3, 3) +
                          mInductorAdmittance * (**mSeriesRes).cast<Complex>())
                             .inverse();
  mSeriesAdmittance = mSeriesSystemInverse * mInductorAdmittance;
  const Matrix capHalf = **mParallelCap / 2.;
  const MatrixComp capReal = (2. / timeStep) * capHalf.cast<Complex>();
  mCapAdmittance = capReal + Complex(0., omega) * capHalf.cast<Complex>();
  mCapPreviousVoltage = capReal - Complex(0., omega) * capHalf.cast<Complex>();
  const MatrixComp shunt =
      mCapAdmittance + ((**mParallelCond) / 2.).cast<Complex>();
  mW = MatrixComp::Zero(6, 6);
  mW.block<3, 3>(0, 0) = mSeriesAdmittance + shunt;
  mW.block<3, 3>(0, 3) = -mSeriesAdmittance;
  mW.block<3, 3>(3, 0) = -mSeriesAdmittance;
  mW.block<3, 3>(3, 3) = mSeriesAdmittance + shunt;
  mDiscreteA.setZero(18, 18);
  mDiscreteB.setZero(18, 6);
}

MatrixComp DP::Ph3::SSN::PiLine::calculateHistoryVector() const {
  mInductorHistory.noalias() =
      mInductorAdmittance * (**mX).block<3, 1>(kSeriesInductorVoltage, 0);
  mInductorHistory +=
      mInductorPreviousCurrent * (**mX).block<3, 1>(kSeriesCurrent, 0);
  mSeriesHistory.noalias() = mSeriesSystemInverse * mInductorHistory;
  mCapHistory0 = -(mCapPreviousVoltage * (**mX).block<3, 1>(kCapVoltage0, 0) +
                   (**mX).block<3, 1>(kCapCurrent0, 0));
  mCapHistory1 = -(mCapPreviousVoltage * (**mX).block<3, 1>(kCapVoltage1, 0) +
                   (**mX).block<3, 1>(kCapCurrent1, 0));
  MatrixComp history = MatrixComp::Zero(6, 1);
  history.block<3, 1>(0, 0) = -mSeriesHistory + mCapHistory0;
  history.block<3, 1>(3, 0) = mSeriesHistory + mCapHistory1;
  return history;
}

void DP::Ph3::SSN::PiLine::updateHistoryVector(MatrixComp &history) {
  history = calculateHistoryVector();
}

void DP::Ph3::SSN::PiLine::updateState(const MatrixComp &,
                                       const MatrixComp &input) {
  const TerminalVoltage v0 = input.block<3, 1>(0, 0);
  const TerminalVoltage v1 = input.block<3, 1>(3, 0);
  const TerminalVoltage voltage = v1 - v0;
  const TerminalVoltage current = mSeriesAdmittance * voltage + mSeriesHistory;
  const TerminalVoltage inductorVoltage = voltage - **mSeriesRes * current;
  const TerminalVoltage cap0Current = mCapAdmittance * v0 + mCapHistory0;
  const TerminalVoltage cap1Current = mCapAdmittance * v1 + mCapHistory1;
  (**mX).block<3, 1>(kSeriesCurrent, 0) = current;
  (**mX).block<3, 1>(kSeriesInductorVoltage, 0) = inductorVoltage;
  (**mX).block<3, 1>(kCapVoltage0, 0) = v0;
  (**mX).block<3, 1>(kCapCurrent0, 0) = cap0Current;
  (**mX).block<3, 1>(kCapVoltage1, 0) = v1;
  (**mX).block<3, 1>(kCapCurrent1, 0) = cap1Current;
}
