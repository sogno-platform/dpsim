// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_TwoTerminalITypeSSNComp.h>
#include <dpsim-models/MNAStampUtils.h>

#include <limits>

using namespace CPS;

DP::Ph3::TwoTerminalITypeSSNComp::TwoTerminalITypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : ITypeSSNComp(uid, name, 3, 3, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp DP::Ph3::TwoTerminalITypeSSNComp::buildInitialInputFromNodes(Real) {
  // Use terminal 1's own line-line RMS voltage and three-phase power.
  // Dividing a difference of terminal powers by the port voltage drop gives
  // the wrong current when both buses have nonzero voltage.
  const Complex voltage = initialSingleVoltage(1);
  if (std::abs(voltage) <= std::numeric_limits<Real>::epsilon())
    return MatrixComp::Zero(3, 1);

  const Complex power = terminal(1)->singlePower();
  const Complex rmsCurrent = std::conj(power / voltage / std::sqrt(3.));
  MatrixComp current = MatrixComp::Zero(3, 1);
  current(0, 0) = RMS_TO_PEAK * rmsCurrent;
  current(1, 0) = current(0, 0) * SHIFT_TO_PHASE_B;
  current(2, 0) = current(0, 0) * SHIFT_TO_PHASE_C;
  return current;
}

void DP::Ph3::TwoTerminalITypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Norton-equivalent admittance G = W^-1 (v = W i + yHist => i = G v - G yHist).
  const MatrixComp &g = mWInverse;

  MNAStampUtils::stampAdmittanceMatrix(
      g, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::TwoTerminalITypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  MatrixComp iHist = -mWInverse * mYHist;

  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             iHist(phase, 0));
  }
  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(1, phase),
                             -iHist(phase, 0));
  }
}

void DP::Ph3::TwoTerminalITypeSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  // i = W^-1 (v - yHist)
  const MatrixComp &voltage = **mIntfVoltage;
  const MatrixComp &g = mWInverse;
  const MatrixComp &history = mYHist;
  MatrixComp &current = **mIntfCurrent;
  const Complex d0 = voltage(0, 0) - history(0, 0);
  const Complex d1 = voltage(1, 0) - history(1, 0);
  const Complex d2 = voltage(2, 0) - history(2, 0);
  current(0, 0) = g(0, 0) * d0 + g(0, 1) * d1 + g(0, 2) * d2;
  current(1, 0) = g(1, 0) * d0 + g(1, 1) * d1 + g(1, 2) * d2;
  current(2, 0) = g(2, 0) * d0 + g(2, 1) * d1 + g(2, 2) * d2;
}

void DP::Ph3::TwoTerminalITypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  (**mIntfVoltage).setZero();

  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < 3; ++phase)
      (**mIntfVoltage)(phase, 0) =
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, phase));
  }
  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < 3; ++phase)
      (**mIntfVoltage)(phase, 0) -=
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, phase));
  }
}
