// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalITypeSSNComp.h>

using namespace CPS;

EMT::Ph3::TwoTerminalITypeSSNComp::TwoTerminalITypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : ITypeSSNComp(uid, name, 3, 3, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp EMT::Ph3::TwoTerminalITypeSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp iInitABC = MatrixComp::Zero(3, 1);

  // Interface current convention:
  // positive current flows from terminal1 to terminal0.
  //
  // No initial terminal current is available here, so initialize with zero.
  return iInitABC;
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
