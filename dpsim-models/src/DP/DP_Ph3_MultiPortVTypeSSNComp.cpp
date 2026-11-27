// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_MultiPortVTypeSSNComp.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

DP::Ph3::MultiPortVTypeSSNComp::MultiPortVTypeSSNComp(String uid, String name,
                                                      Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 6, 6, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
}

MatrixComp DP::Ph3::MultiPortVTypeSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp input = MatrixComp::Zero(6, 1);
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    const Complex phaseA = RMS3PH_TO_PEAK1PH * initialSingleVoltage(terminal);
    input(3 * terminal, 0) = phaseA;
    input(3 * terminal + 1, 0) = phaseA * SHIFT_TO_PHASE_B;
    input(3 * terminal + 2, 0) = phaseA * SHIFT_TO_PHASE_C;
  }
  return input;
}

void DP::Ph3::MultiPortVTypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (UInt rowTerminal = 0; rowTerminal < 2; ++rowTerminal) {
    if (!terminalNotGrounded(rowTerminal))
      continue;
    for (UInt rowPhase = 0; rowPhase < 3; ++rowPhase) {
      const UInt row = 3 * rowTerminal + rowPhase;
      for (UInt colTerminal = 0; colTerminal < 2; ++colTerminal) {
        if (!terminalNotGrounded(colTerminal))
          continue;
        for (UInt colPhase = 0; colPhase < 3; ++colPhase) {
          const UInt col = 3 * colTerminal + colPhase;
          Math::addToMatrixElement(
              systemMatrix, matrixNodeIndex(rowTerminal, rowPhase),
              matrixNodeIndex(colTerminal, colPhase), mW(row, col));
        }
      }
    }
  }
}

void DP::Ph3::MultiPortVTypeSSNComp::mnaCompApplyRightSideVectorStamp(
    RhsVector &rightVector) {
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    if (!terminalNotGrounded(terminal))
      continue;
    for (UInt phase = 0; phase < 3; ++phase) {
      const UInt port = 3 * terminal + phase;
      Math::setVectorElement(rightVector, matrixNodeIndex(terminal, phase),
                             -mYHist(port, 0));
    }
  }
}

void DP::Ph3::MultiPortVTypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  (**mIntfVoltage).setZero();
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    if (!terminalNotGrounded(terminal))
      continue;
    for (UInt phase = 0; phase < 3; ++phase) {
      (**mIntfVoltage)(3 * terminal + phase, 0) =
          Math::complexFromVectorElement(leftVector,
                                         matrixNodeIndex(terminal, phase));
    }
  }
}
