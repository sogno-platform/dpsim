// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_TwoTerminalVTypeSSNComp.h>

using namespace CPS;

EMT::DC::TwoTerminalVTypeSSNComp::TwoTerminalVTypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 1, 1, logLevel) {
  mPhaseType = PhaseType::DC;
  setTerminalNumber(2);
}

void EMT::DC::TwoTerminalVTypeSSNComp::validateDCTerminals() const {
  for (UInt terminalIdx = 0; terminalIdx < 2; ++terminalIdx) {
    const auto terminalNode =
        const_cast<TwoTerminalVTypeSSNComp *>(this)->node(terminalIdx);
    if (!terminalNode->isGround() && terminalNode->phaseType() != PhaseType::DC)
      throw std::invalid_argument(
          "DC SSN components require DC nodes or ground at both terminals.");
  }
}

MatrixComp EMT::DC::TwoTerminalVTypeSSNComp::buildInitialInputFromNodes(Real) {
  validateDCTerminals();
  const Complex voltage = initialSingleVoltage(1) - initialSingleVoltage(0);
  if (!Math::isFinite(voltage) ||
      std::abs(voltage.imag()) > std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC initial node voltages must be finite real scalar values.");

  MatrixComp result(1, 1);
  result(0, 0) = voltage.real();
  return result;
}

void EMT::DC::TwoTerminalVTypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  validateDCTerminals();
  MNAStampUtils::stampConductance(mW(0, 0), systemMatrix, matrixNodeIndex(0),
                                  matrixNodeIndex(1), terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
}

void EMT::DC::TwoTerminalVTypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0), mYHist(0, 0));
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1), -mYHist(0, 0));
}

void EMT::DC::TwoTerminalVTypeSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0.0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0));

  if (!Math::isFinite((**mIntfVoltage)(0, 0)))
    throw std::runtime_error(
        "DC SSN voltage update produced a non-finite value.");
}
