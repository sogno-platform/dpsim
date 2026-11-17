/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph3_SSN_PiLine.h>
using namespace CPS;

namespace {
constexpr Int kSeriesCurrent = 0;
constexpr Int kSeriesInductorVoltage = 3;
constexpr Int kCapVoltage0 = 6;
constexpr Int kCapCurrent0 = 9;
constexpr Int kCapVoltage1 = 12;
constexpr Int kCapCurrent1 = 15;
} // namespace

EMT::Ph3::SSN::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : MultiPortVTypeSSNComp(uid, name, logLevel),
      Base::Ph3::PiLine(mAttributes), mStateValue(&mX->get()),
      mInductorAdmittance(Matrix::Zero(3, 3)),
      mSeriesAdmittance(Matrix::Zero(3, 3)),
      mSeriesSystemInverse(Matrix::Identity(3, 3)),
      mCapAdmittance(Matrix::Zero(3, 3)), mSeriesHistory(Matrix::Zero(3, 1)),
      mInductorHistory(Matrix::Zero(3, 1)), mCapHistory0(Matrix::Zero(3, 1)),
      mCapHistory1(Matrix::Zero(3, 1)) {
  *mStateValue = Matrix::Zero(18, 1);
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::PiLine::clone(String copySuffix) {
  auto copy = SharedFactory<PiLine>::make(copySuffix, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void EMT::Ph3::SSN::PiLine::setParameters(const Matrix &seriesResistance,
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
  EMT::SSNComp::setParameters(Matrix::Zero(18, 18), Matrix::Zero(18, 6),
                              Matrix::Zero(6, 18), Matrix::Zero(6, 6));
}

void EMT::Ph3::SSN::PiLine::initializeFromNodesAndTerminals(Real frequency) {
  if ((**mParallelCond)(0, 0) <= 0.) {
    **mParallelCond = Matrix::Zero(3, 3);
    (**mParallelCond).diagonal().setConstant(1e-6);
  }

  const Real omega = 2. * PI * frequency;
  MatrixComp v0Phasor = MatrixComp::Zero(3, 1);
  MatrixComp v1Phasor = MatrixComp::Zero(3, 1);
  v0Phasor(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  v1Phasor(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1);
  v0Phasor(1, 0) = v0Phasor(0, 0) * SHIFT_TO_PHASE_B;
  v0Phasor(2, 0) = v0Phasor(0, 0) * SHIFT_TO_PHASE_C;
  v1Phasor(1, 0) = v1Phasor(0, 0) * SHIFT_TO_PHASE_B;
  v1Phasor(2, 0) = v1Phasor(0, 0) * SHIFT_TO_PHASE_C;

  const MatrixComp voltagePhasor = v1Phasor - v0Phasor;
  const MatrixComp impedance =
      (**mSeriesRes).cast<Complex>() +
      Complex(0., omega) * (**mSeriesInd).cast<Complex>();
  const MatrixComp currentPhasor = impedance.inverse() * voltagePhasor;
  const MatrixComp inductorVoltagePhasor =
      voltagePhasor - (**mSeriesRes).cast<Complex>() * currentPhasor;
  const MatrixComp capHalf = ((**mParallelCap) / 2.).cast<Complex>();
  const MatrixComp capCurrent0 = Complex(0., omega) * capHalf * v0Phasor;
  const MatrixComp capCurrent1 = Complex(0., omega) * capHalf * v1Phasor;

  *mIntfVoltageValue = Matrix::Zero(6, 1);
  mIntfVoltageValue->block(0, 0, 3, 1) = v0Phasor.real();
  mIntfVoltageValue->block(3, 0, 3, 1) = v1Phasor.real();
  *mIntfCurrentValue = Matrix::Zero(6, 1);
  mIntfCurrentValue->block(0, 0, 3, 1) =
      -currentPhasor.real() + capCurrent0.real();
  mIntfCurrentValue->block(3, 0, 3, 1) =
      currentPhasor.real() + capCurrent1.real();
  mStateValue->setZero();
  mStateValue->block<3, 1>(kSeriesCurrent, 0) = currentPhasor.real();
  mStateValue->block<3, 1>(kSeriesInductorVoltage, 0) =
      inductorVoltagePhasor.real();
  mStateValue->block<3, 1>(kCapVoltage0, 0) = v0Phasor.real();
  mStateValue->block<3, 1>(kCapCurrent0, 0) = capCurrent0.real();
  mStateValue->block<3, 1>(kCapVoltage1, 0) = v1Phasor.real();
  mStateValue->block<3, 1>(kCapCurrent1, 0) = capCurrent1.real();
}

void EMT::Ph3::SSN::PiLine::recomputeDiscreteModel() {
  const Real timeStep = mTimeStep;
  mInductorAdmittance = (timeStep / 2.) * (**mSeriesInd).inverse();
  mSeriesSystemInverse =
      (Matrix::Identity(3, 3) + mInductorAdmittance * **mSeriesRes).inverse();
  mSeriesAdmittance = mSeriesSystemInverse * mInductorAdmittance;
  mCapAdmittance = (**mParallelCap) / timeStep;
  const Matrix shunt = mCapAdmittance + (**mParallelCond) / 2.;
  mW = Matrix::Zero(6, 6);
  mW.block<3, 3>(0, 0) = mSeriesAdmittance + shunt;
  mW.block<3, 3>(0, 3) = -mSeriesAdmittance;
  mW.block<3, 3>(3, 0) = -mSeriesAdmittance;
  mW.block<3, 3>(3, 3) = mSeriesAdmittance + shunt;
  mdA.setZero(18, 18);
  mdB.setZero(18, 6);
}

void EMT::Ph3::SSN::PiLine::calculateHistoryVector(Matrix &history) const {
  mInductorHistory.noalias() =
      mInductorAdmittance * mStateValue->block<3, 1>(kSeriesInductorVoltage, 0);
  mInductorHistory += mStateValue->block<3, 1>(kSeriesCurrent, 0);
  mSeriesHistory.noalias() = mSeriesSystemInverse * mInductorHistory;
  mCapHistory0 = -mStateValue->block<3, 1>(kCapCurrent0, 0) -
                 mCapAdmittance * mStateValue->block<3, 1>(kCapVoltage0, 0);
  mCapHistory1 = -mStateValue->block<3, 1>(kCapCurrent1, 0) -
                 mCapAdmittance * mStateValue->block<3, 1>(kCapVoltage1, 0);
  history.resize(6, 1);
  history.block(0, 0, 3, 1) = -mSeriesHistory + mCapHistory0;
  history.block(3, 0, 3, 1) = mSeriesHistory + mCapHistory1;
}

void EMT::Ph3::SSN::PiLine::updateState(const Matrix &, const Matrix &input) {
  const TerminalVoltage v0 = input.block(0, 0, 3, 1);
  const TerminalVoltage v1 = input.block(3, 0, 3, 1);
  const TerminalVoltage interfaceVoltage = v1 - v0;
  const TerminalVoltage seriesCurrent =
      mSeriesAdmittance * interfaceVoltage + mSeriesHistory;
  const TerminalVoltage inductorVoltage =
      interfaceVoltage - **mSeriesRes * seriesCurrent;
  const TerminalVoltage capCurrent0 = mCapAdmittance * v0 + mCapHistory0;
  const TerminalVoltage capCurrent1 = mCapAdmittance * v1 + mCapHistory1;
  mStateValue->block<3, 1>(kSeriesCurrent, 0) = seriesCurrent;
  mStateValue->block<3, 1>(kSeriesInductorVoltage, 0) = inductorVoltage;
  mStateValue->block<3, 1>(kCapVoltage0, 0) = v0;
  mStateValue->block<3, 1>(kCapCurrent0, 0) = capCurrent0;
  mStateValue->block<3, 1>(kCapVoltage1, 0) = v1;
  mStateValue->block<3, 1>(kCapCurrent1, 0) = capCurrent1;
}
