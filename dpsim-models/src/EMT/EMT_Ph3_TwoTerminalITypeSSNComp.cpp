// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalITypeSSNComp.h>

#include <limits>

using namespace CPS;

EMT::Ph3::TwoTerminalITypeSSNComp::TwoTerminalITypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : ITypeSSNComp(uid, name, 3, 3, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp EMT::Ph3::TwoTerminalITypeSSNComp::buildInitialInputFromNodes(Real) {
  // Reconstruct the operating current from the PF terminal voltage and power.
  // The EMT SSN input uses phase-peak phasors before taking the t=0 real part.
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

void EMT::Ph3::TwoTerminalITypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  Matrix G = mW.inverse();

  MNAStampUtils::stampConductanceMatrix(
      G, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void EMT::Ph3::TwoTerminalITypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  Matrix G = mW.inverse();

  // i = G * v - G * mYHist
  Matrix iHist = -G * mYHist;

  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < 3; ++phase) {
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             iHist(phase, 0));
    }
  }

  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < 3; ++phase) {
      Math::setVectorElement(rightVector, matrixNodeIndex(1, phase),
                             -iHist(phase, 0));
    }
  }
}

void EMT::Ph3::TwoTerminalITypeSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  Matrix G = mW.inverse();

  // I-type Norton-equivalent current:
  //
  // v = mW * i + mYHist
  // => i = mW^{-1} * (v - mYHist)
  **mIntfCurrent = G * (**mIntfVoltage - mYHist);
}

void EMT::Ph3::TwoTerminalITypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // Interface voltage convention: v = v_terminal1 - v_terminal0
  **mIntfVoltage = Matrix::Zero(3, 1);

  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < 3; ++phase) {
      (**mIntfVoltage)(phase, 0) =
          Math::realFromVectorElement(leftVector, matrixNodeIndex(1, phase));
    }
  }

  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < 3; ++phase) {
      (**mIntfVoltage)(phase, 0) -=
          Math::realFromVectorElement(leftVector, matrixNodeIndex(0, phase));
    }
  }
}
