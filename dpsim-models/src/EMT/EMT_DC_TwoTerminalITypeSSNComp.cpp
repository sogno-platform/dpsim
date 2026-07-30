// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_TwoTerminalITypeSSNComp.h>

using namespace CPS;

EMT::DC::TwoTerminalITypeSSNComp::TwoTerminalITypeSSNComp(
    String uid, String name, Logger::Level logLevel)
    : ITypeSSNComp(uid, name, 1, 1, logLevel) {
  mPhaseType = PhaseType::DC;
  setTerminalNumber(2);
}

void EMT::DC::TwoTerminalITypeSSNComp::validateDCTerminals() const {
  for (UInt terminalIdx = 0; terminalIdx < 2; ++terminalIdx) {
    const auto terminalNode =
        const_cast<TwoTerminalITypeSSNComp *>(this)->node(terminalIdx);
    if (!terminalNode->isGround() && terminalNode->phaseType() != PhaseType::DC)
      throw std::invalid_argument(
          "DC SSN components require DC nodes or ground at both terminals.");
  }
}

Real EMT::DC::TwoTerminalITypeSSNComp::companionConductance() const {
  if (mW.rows() != 1 || mW.cols() != 1 || !Math::isFinite(mW(0, 0)) ||
      std::abs(mW(0, 0)) <= std::numeric_limits<Real>::epsilon())
    throw std::runtime_error(
        "DC I-type SSN component has a singular companion impedance.");
  const Real conductance = 1.0 / mW(0, 0);
  if (!Math::isFinite(conductance))
    throw std::runtime_error(
        "DC I-type SSN companion conductance is non-finite.");
  return conductance;
}

MatrixComp EMT::DC::TwoTerminalITypeSSNComp::buildInitialInputFromNodes(Real) {
  validateDCTerminals();
  return MatrixComp::Zero(1, 1);
}

void EMT::DC::TwoTerminalITypeSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  validateDCTerminals();
  MNAStampUtils::stampConductance(companionConductance(), systemMatrix,
                                  matrixNodeIndex(0), matrixNodeIndex(1),
                                  terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
}

void EMT::DC::TwoTerminalITypeSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  const Real historyCurrent = -companionConductance() * mYHist(0, 0);
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0), historyCurrent);
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1), -historyCurrent);
}

void EMT::DC::TwoTerminalITypeSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  (**mIntfCurrent)(0, 0) =
      companionConductance() * ((**mIntfVoltage)(0, 0) - mYHist(0, 0));
  if (!Math::isFinite((**mIntfCurrent)(0, 0)))
    throw std::runtime_error(
        "DC SSN current update produced a non-finite value.");
}

void EMT::DC::TwoTerminalITypeSSNComp::mnaCompUpdateVoltage(
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
