// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_TwoTerminalVTypeSplitSSNComp.h>
#include <dpsim-models/MNAStampUtils.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

DP::Ph3::TwoTerminalVTypeSplitSSNComp::TwoTerminalVTypeSplitSSNComp(
    String uid, String name, Int controllerStateSize, Int controllerInputSize,
    Int controllerOutputSize, Logger::Level logLevel)
    : VTypeSplitSSNComp(uid, name, 6, 6, controllerStateSize,
                        controllerInputSize, controllerOutputSize, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp
DP::Ph3::TwoTerminalVTypeSplitSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp result = MatrixComp::Zero(3, 1);
  result(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  result(1, 0) = result(0, 0) * SHIFT_TO_PHASE_B;
  result(2, 0) = result(0, 0) * SHIFT_TO_PHASE_C;
  return result;
}

void DP::Ph3::TwoTerminalVTypeSplitSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mW, 3, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::TwoTerminalVTypeSplitSSNComp::mnaCompApplyRightSideVectorStamp(
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

void DP::Ph3::TwoTerminalVTypeSplitSSNComp::mnaCompUpdateVoltage(
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
