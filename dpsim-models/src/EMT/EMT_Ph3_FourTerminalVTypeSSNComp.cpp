// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_FourTerminalVTypeSSNComp.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

EMT::Ph3::FourTerminalVTypeSSNComp::FourTerminalVTypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : EMT::VTypeSSNComp(uid, name, 6, 6, logLevel) {

  mPhaseType = PhaseType::ABC;
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
EMT::Ph3::FourTerminalVTypeSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp vInitABC = MatrixComp::Zero(6, 1);

  // Interface voltage convention:
  // u = [v_terminal2 - v_terminal0;
  //      v_terminal3 - v_terminal1]

  // First three-phase interface voltage: v20 = v2 - v0
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(2) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  // Second three-phase interface voltage: v31 = v3 - v1
  vInitABC(3, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(3) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(1);
  vInitABC(4, 0) = vInitABC(3, 0) * SHIFT_TO_PHASE_B;
  vInitABC(5, 0) = vInitABC(3, 0) * SHIFT_TO_PHASE_C;

  return vInitABC;
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
