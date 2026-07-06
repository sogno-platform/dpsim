// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_TwoTerminalVTypeSSNComp.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph3::TwoTerminalVTypeSSNComp::TwoTerminalVTypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 3, 3, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp DP::Ph3::TwoTerminalVTypeSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp vInit = MatrixComp::Zero(3, 1);

  // v = v_terminal1 - v_terminal0 (DP envelope phasor), balanced default.
  vInit(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  vInit(1, 0) = vInit(0, 0) * SHIFT_TO_PHASE_B;
  vInit(2, 0) = vInit(0, 0) * SHIFT_TO_PHASE_C;

  return vInit;
}

void DP::Ph3::TwoTerminalVTypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mW, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::TwoTerminalVTypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             mYHist(phase, 0));
  }
  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(1, phase),
                             -mYHist(phase, 0));
  }
}

void DP::Ph3::TwoTerminalVTypeSSNComp::mnaCompUpdateVoltage(
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
