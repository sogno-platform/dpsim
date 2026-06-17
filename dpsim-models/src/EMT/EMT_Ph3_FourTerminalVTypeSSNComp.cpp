// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_FourTerminalVTypeSSNComp.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

EMT::Ph3::FourTerminalVTypeSSNComp::FourTerminalVTypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : EMT::VTypeSSNComp(uid, name, 6, 6, logLevel) {

  setTerminalNumber(4);

  const Matrix identity3 = Matrix::Identity(3, 3);

  mTerminalIncidence = Matrix::Zero(6, 12);

  // Terminal layout:
  //
  //   v0   v1
  //   v2   v3
  //
  // Interface voltages:
  //
  //   vin  = v2 - v0
  //   vout = v3 - v1

  mTerminalIncidence.block(0, 0, 3, 3) = -identity3;
  mTerminalIncidence.block(0, 6, 3, 3) = identity3;

  mTerminalIncidence.block(3, 3, 3, 3) = -identity3;
  mTerminalIncidence.block(3, 9, 3, 3) = identity3;
}

const Matrix &
EMT::Ph3::FourTerminalVTypeSSNComp::terminalIncidenceMatrix() const {
  return mTerminalIncidence;
}

void EMT::Ph3::FourTerminalVTypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {

  Matrix vTerm = Matrix::Zero(12, 1);

  for (UInt terminal = 0; terminal < 4; ++terminal) {
    if (!terminalNotGrounded(terminal))
      continue;

    for (UInt phase = 0; phase < 3; ++phase) {
      vTerm(3 * terminal + phase, 0) = Math::realFromVectorElement(
          leftVector, matrixNodeIndex(terminal, phase));
    }
  }

  **mIntfVoltage = mTerminalIncidence * vTerm;
}

MatrixComp
EMT::Ph3::FourTerminalVTypeSSNComp::buildInitialInputFromNodes(Real frequency) {

  return MatrixComp::Zero(6, 1);
}

void EMT::Ph3::FourTerminalVTypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {

  const Matrix yTerm = mTerminalIncidence.transpose() * mW * mTerminalIncidence;

  for (UInt terminalRow = 0; terminalRow < 4; ++terminalRow) {
    if (!terminalNotGrounded(terminalRow))
      continue;

    for (UInt phaseRow = 0; phaseRow < 3; ++phaseRow) {
      const UInt row = 3 * terminalRow + phaseRow;

      for (UInt terminalCol = 0; terminalCol < 4; ++terminalCol) {
        if (!terminalNotGrounded(terminalCol))
          continue;

        for (UInt phaseCol = 0; phaseCol < 3; ++phaseCol) {
          const UInt col = 3 * terminalCol + phaseCol;

          Math::addToMatrixElement(
              systemMatrix, matrixNodeIndex(terminalRow, phaseRow),
              matrixNodeIndex(terminalCol, phaseCol), yTerm(row, col));
        }
      }
    }
  }
}

void EMT::Ph3::FourTerminalVTypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {

  const Matrix iHistTerm = mTerminalIncidence.transpose() * mYHist;

  for (UInt terminal = 0; terminal < 4; ++terminal) {
    if (!terminalNotGrounded(terminal))
      continue;

    for (UInt phase = 0; phase < 3; ++phase) {
      const UInt idx = 3 * terminal + phase;

      Math::setVectorElement(rightVector, matrixNodeIndex(terminal, phase),
                             -iHistTerm(idx, 0));
    }
  }
}
