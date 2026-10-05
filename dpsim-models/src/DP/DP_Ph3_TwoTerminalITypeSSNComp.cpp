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
  // PF stores line-line RMS voltage and terminal RMS power. Convert their
  // operating point into the phase-peak complex-envelope input used by DP.
  const Complex voltage = initialSingleVoltage(1) - initialSingleVoltage(0);
  if (std::abs(voltage) <= std::numeric_limits<Real>::epsilon())
    return MatrixComp::Zero(3, 1);

  const Complex power = terminal(1)->singlePower() - terminal(0)->singlePower();
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
  MatrixComp g = mW.inverse();

  MNAStampUtils::stampAdmittanceMatrix(
      g, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::TwoTerminalITypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  MatrixComp iHist = -mW.inverse() * mYHist;

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
  **mIntfCurrent = mW.inverse() * (**mIntfVoltage - mYHist);
}

void DP::Ph3::TwoTerminalITypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  **mIntfVoltage = MatrixComp::Zero(3, 1);

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
