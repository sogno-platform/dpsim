// SPDX-FileCopyrightText: 2025 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph1_SSN_Full_Serial_RLC.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

EMT::Ph1::SSN::Full_Serial_RLC::Full_Serial_RLC(String uid, String name,
                                                Logger::Level logLevel)
    : VTypeSSNComp(uid, name, 1, 1, logLevel), Base::Ph1::Resistor(mAttributes),
      Base::Ph1::Inductor(mAttributes), Base::Ph1::Capacitor(mAttributes) {
  mPhaseType = PhaseType::Single;
  setTerminalNumber(2);
}

SimPowerComp<Real>::Ptr EMT::Ph1::SSN::Full_Serial_RLC::clone(String name) {
  auto copy = Full_Serial_RLC::make(name, mLogLevel);
  copy->setParameters(**mResistance, **mInductance, **mCapacitance);
  return copy;
}

void EMT::Ph1::SSN::Full_Serial_RLC::setParameters(Real resistance,
                                                   Real inductance,
                                                   Real capacitance) {
  Base::Ph1::Resistor::setParameters(resistance);
  Base::Ph1::Inductor::setParameters(inductance);
  Base::Ph1::Capacitor::setParameters(capacitance);

  Matrix a = Matrix::Zero(2, 2);
  a(0, 1) = 1. / capacitance;
  a(1, 0) = -1. / inductance;
  a(1, 1) = -resistance / inductance;
  Matrix b = Matrix::Zero(2, 1);
  b(1, 0) = 1. / inductance;
  Matrix c = Matrix::Zero(1, 2);
  c(0, 1) = 1.;
  SSNComp::setParameters(a, b, c, Matrix::Zero(1, 1));
}

MatrixComp EMT::Ph1::SSN::Full_Serial_RLC::buildInitialInputFromNodes(Real) {
  MatrixComp voltage(1, 1);
  voltage(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  return voltage;
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampConductance(mW(0, 0), systemMatrix, matrixNodeIndex(0),
                                  matrixNodeIndex(1), terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), mYHist(0, 0));
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), -mYHist(0, 0));
}

Bool EMT::Ph1::SSN::Full_Serial_RLC::getRightVectorStampIndices(
    std::vector<UInt> &indices) {
  indices.clear();
  if (terminalNotGrounded(0))
    indices.push_back(matrixNodeIndex(0, 0));
  if (terminalNotGrounded(1))
    indices.push_back(matrixNodeIndex(1, 0));
  return true;
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  mIntfVoltageValue->setZero();
  if (terminalNotGrounded(1))
    (*mIntfVoltageValue)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  if (terminalNotGrounded(0))
    (*mIntfVoltageValue)(0, 0) -=
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
}
