// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeVariableSSNComp.h>

using namespace CPS;

EMT::Ph3::TwoTerminalVTypeVariableSSNComp::TwoTerminalVTypeVariableSSNComp(
    String uid, String name, Logger::Level logLevel)
    : VTypeVariableSSNComp(uid, name, 3, 3, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp
EMT::Ph3::TwoTerminalVTypeVariableSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp vInitABC = MatrixComp::Zero(3, 1);

  // Interface voltage convention: v = v_terminal1 - v_terminal0
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  return vInitABC;
}

void EMT::Ph3::TwoTerminalVTypeVariableSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductanceMatrix(
      mW, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void EMT::Ph3::TwoTerminalVTypeVariableSSNComp::
    mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
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

void EMT::Ph3::TwoTerminalVTypeVariableSSNComp::mnaCompUpdateVoltage(
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
