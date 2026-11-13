/* SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph3_SSN_PiLine.h>
#include <dpsim-models/MNAStampUtils.h>

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
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      Base::Ph3::PiLine(mAttributes),
      mState(mAttributes->create<MatrixComp>("state")),
      mInductorAdmittance(MatrixComp::Zero(3, 3)),
      mSeriesAdmittance(MatrixComp::Zero(3, 3)),
      mSeriesSystemInverse(MatrixComp::Identity(3, 3)),
      mCapAdmittance(MatrixComp::Zero(3, 3)),
      mCapPreviousVoltage(MatrixComp::Zero(3, 3)),
      mSeriesHistory(MatrixComp::Zero(3, 1)),
      mInductorHistory(MatrixComp::Zero(3, 1)),
      mCapHistory0(MatrixComp::Zero(3, 1)),
      mCapHistory1(MatrixComp::Zero(3, 1)) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
  **mState = MatrixComp::Zero(18, 1);
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

  **mIntfVoltage = voltage;
  **mIntfCurrent = current;
  (**mState).setZero();
  (**mState).block<3, 1>(kSeriesCurrent, 0) = current;
  (**mState).block<3, 1>(kSeriesInductorVoltage, 0) = inductorVoltage;
  (**mState).block<3, 1>(kCapVoltage0, 0) = v0;
  (**mState).block<3, 1>(kCapCurrent0, 0) = cap0Current;
  (**mState).block<3, 1>(kCapVoltage1, 0) = v1;
  (**mState).block<3, 1>(kCapCurrent1, 0) = cap1Current;
}

void DP::Ph3::SSN::PiLine::mnaCompInitialize(Real omega, Real timeStep,
                                             Attribute<Matrix>::Ptr) {
  updateMatrixNodeIndices();

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
}

void DP::Ph3::SSN::PiLine::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mSeriesAdmittance, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
  const MatrixComp shunt =
      mCapAdmittance + ((**mParallelCond) / 2.).cast<Complex>();
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    MNAStampUtils::stampAdmittanceMatrix(
        shunt, systemMatrix, matrixNodeIndex(terminal), 0,
        terminalNotGrounded(terminal), false, mSLog);
  }
}

void DP::Ph3::SSN::PiLine::mnaCompPreStep(Real, Int) {
  mInductorHistory.noalias() =
      mInductorAdmittance * (**mState).block<3, 1>(kSeriesInductorVoltage, 0);
  mInductorHistory +=
      mInductorPreviousCurrent * (**mState).block<3, 1>(kSeriesCurrent, 0);
  mSeriesHistory.noalias() = mSeriesSystemInverse * mInductorHistory;
  mCapHistory0 = -(**mState).block<3, 1>(kCapCurrent0, 0) -
                 mCapPreviousVoltage * (**mState).block<3, 1>(kCapVoltage0, 0);
  mCapHistory1 = -(**mState).block<3, 1>(kCapCurrent1, 0) -
                 mCapPreviousVoltage * (**mState).block<3, 1>(kCapVoltage1, 0);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::SSN::PiLine::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             mSeriesHistory(phase, 0) - mCapHistory0(phase, 0));
  }
  if (terminalNotGrounded(1)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(1, phase),
                             -mSeriesHistory(phase, 0) -
                                 mCapHistory1(phase, 0));
  }
}

void DP::Ph3::SSN::PiLine::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::addToVectorElement(rightVector, matrixNodeIndex(0, phase),
                               Math::complexFromVectorElement(
                                   **mRightVector, matrixNodeIndex(0, phase)));
  }
  if (terminalNotGrounded(1)) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::addToVectorElement(rightVector, matrixNodeIndex(1, phase),
                               Math::complexFromVectorElement(
                                   **mRightVector, matrixNodeIndex(1, phase)));
  }
}

DP::Ph3::SSN::PiLine::TerminalVoltage
DP::Ph3::SSN::PiLine::terminalVoltage(const Matrix &leftVector, UInt terminal) {
  TerminalVoltage voltage = TerminalVoltage::Zero();
  if (terminalNotGrounded(terminal)) {
    for (UInt phase = 0; phase < 3; ++phase)
      voltage(phase, 0) = Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(terminal, phase));
  }
  return voltage;
}

void DP::Ph3::SSN::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mIntfVoltage =
      terminalVoltage(leftVector, 1) - terminalVoltage(leftVector, 0);
}

void DP::Ph3::SSN::PiLine::mnaCompUpdateCurrent(const Matrix &) {
  **mIntfCurrent = mSeriesAdmittance * (**mIntfVoltage) + mSeriesHistory;
}

void DP::Ph3::SSN::PiLine::mnaCompPostStep(Real, Int,
                                           Attribute<Matrix>::Ptr &leftVector) {
  // Reuse the terminal voltages for both the interface update and the
  // capacitor state update instead of extracting them twice.
  const MatrixComp v0 = terminalVoltage(**leftVector, 0);
  const MatrixComp v1 = terminalVoltage(**leftVector, 1);
  **mIntfVoltage = v1 - v0;
  (**mIntfCurrent).noalias() = mSeriesAdmittance * (**mIntfVoltage);
  **mIntfCurrent += mSeriesHistory;

  const TerminalVoltage inductorVoltage =
      **mIntfVoltage - (**mSeriesRes).cast<Complex>() * (**mIntfCurrent);
  const TerminalVoltage capCurrent0 = mCapAdmittance * v0 + mCapHistory0;
  const TerminalVoltage capCurrent1 = mCapAdmittance * v1 + mCapHistory1;

  (**mState).block<3, 1>(kSeriesCurrent, 0) = **mIntfCurrent;
  (**mState).block<3, 1>(kSeriesInductorVoltage, 0) = inductorVoltage;
  (**mState).block<3, 1>(kCapVoltage0, 0) = v0;
  (**mState).block<3, 1>(kCapCurrent0, 0) = capCurrent0;
  (**mState).block<3, 1>(kCapVoltage1, 0) = v1;
  (**mState).block<3, 1>(kCapCurrent1, 0) = capCurrent1;
}

void DP::Ph3::SSN::PiLine::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mState);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph3::SSN::PiLine::mnaCompAddPostStepDependencies(
    AttributeBase::List &, AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mState);
}
