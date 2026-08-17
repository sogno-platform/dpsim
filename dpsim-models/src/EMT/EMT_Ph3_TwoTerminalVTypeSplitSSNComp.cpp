// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeSplitSSNComp.h>

using namespace CPS;

EMT::Ph3::TwoTerminalVTypeSplitSSNComp::TwoTerminalVTypeSplitSSNComp(
    String uid, String name, Int controllerStateSize, Int controllerInputSize,
    Int controllerOutputSize, Logger::Level logLevel)
    : VTypeSplitSSNComp(uid, name, 3, 3, controllerStateSize,
                        controllerInputSize, controllerOutputSize, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp
EMT::Ph3::TwoTerminalVTypeSplitSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp vInitABC = MatrixComp::Zero(3, 1);

  // Interface voltage convention: v = v_terminal1 - v_terminal0
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  return vInitABC;
}

void EMT::Ph3::TwoTerminalVTypeSplitSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductanceMatrix(
      mW, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void EMT::Ph3::TwoTerminalVTypeSplitSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < 3; ++phase) {
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             mYHist(phase, 0));
    }
  }

  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < 3; ++phase) {
      Math::setVectorElement(rightVector, matrixNodeIndex(1, phase),
                             -mYHist(phase, 0));
    }
  }
}

void EMT::Ph3::TwoTerminalVTypeSplitSSNComp::mnaCompUpdateVoltage(
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
