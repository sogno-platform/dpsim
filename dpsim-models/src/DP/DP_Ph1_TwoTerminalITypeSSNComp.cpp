// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph1_TwoTerminalITypeSSNComp.h>
#include <dpsim-models/MNAStampUtils.h>

#include <limits>

using namespace CPS;

DP::Ph1::TwoTerminalITypeSSNComp::TwoTerminalITypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : ITypeSSNComp(uid, name, 1, 1, logLevel) {
  mPhaseType = PhaseType::Single;
  setTerminalNumber(2);
}

MatrixComp DP::Ph1::TwoTerminalITypeSSNComp::buildInitialInputFromNodes(Real) {
  // Recover the operating current from the power-flow data on the terminals.
  // The I-type SSN input is oriented from terminal 1 to terminal 0, matching
  // the voltage convention used by the port model (v1 - v0).
  const Complex voltage = initialSingleVoltage(1) - initialSingleVoltage(0);
  if (std::abs(voltage) <= std::numeric_limits<Real>::epsilon())
    return MatrixComp::Zero(1, 1);

  const Complex power = terminal(1)->singlePower() - terminal(0)->singlePower();
  MatrixComp current = MatrixComp::Zero(1, 1);
  current(0, 0) = std::conj(power / voltage);
  return current;
}

void DP::Ph1::TwoTerminalITypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Norton-equivalent admittance G = W^-1 (v = W i + yHist => i = G v - G yHist).
  Complex g = Complex(1., 0.) / mW(0, 0);
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    MNAStampUtils::stampAdmittance(
        g, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
        terminalNotGrounded(0), terminalNotGrounded(1), mSLog, mNumFreqs, freq);
  }
}

void DP::Ph1::TwoTerminalITypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  Complex iHist = -mYHist(0, 0) / mW(0, 0);
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    if (terminalNotGrounded(0))
      Math::setVectorElement(rightVector, matrixNodeIndex(0), iHist, mNumFreqs,
                             freq);
    if (terminalNotGrounded(1))
      Math::setVectorElement(rightVector, matrixNodeIndex(1), -iHist, mNumFreqs,
                             freq);
  }
}

void DP::Ph1::TwoTerminalITypeSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  // i = W^-1 (v - yHist)
  **mIntfCurrent = mW.inverse() * (**mIntfVoltage - mYHist);
}

void DP::Ph1::TwoTerminalITypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // v = v_terminal1 - v_terminal0 (DP envelope phasor)
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    (**mIntfVoltage)(0, freq) = 0;
    if (terminalNotGrounded(1))
      (**mIntfVoltage)(0, freq) = Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(1), mNumFreqs, freq);
    if (terminalNotGrounded(0))
      (**mIntfVoltage)(0, freq) -= Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(0), mNumFreqs, freq);
  }
}
