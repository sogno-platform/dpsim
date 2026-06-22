// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph1_TwoTerminalVTypeSSNComp.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::TwoTerminalVTypeSSNComp::TwoTerminalVTypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 1, 1, logLevel) {
  mPhaseType = PhaseType::Single;
  setTerminalNumber(2);
}

MatrixComp DP::Ph1::TwoTerminalVTypeSSNComp::buildInitialInputFromNodes(Real) {
  // v = v_terminal1 - v_terminal0 (DP envelope phasor)
  MatrixComp vInit = MatrixComp::Zero(1, 1);
  vInit(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  return vInit;
}

void DP::Ph1::TwoTerminalVTypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    MNAStampUtils::stampAdmittance(
        mW(0, 0), systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
        terminalNotGrounded(0), terminalNotGrounded(1), mSLog, mNumFreqs, freq);
  }
}

void DP::Ph1::TwoTerminalVTypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    if (terminalNotGrounded(0))
      Math::setVectorElement(rightVector, matrixNodeIndex(0), mYHist(0, 0),
                             mNumFreqs, freq);
    if (terminalNotGrounded(1))
      Math::setVectorElement(rightVector, matrixNodeIndex(1), -mYHist(0, 0),
                             mNumFreqs, freq);
  }
}

void DP::Ph1::TwoTerminalVTypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
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
