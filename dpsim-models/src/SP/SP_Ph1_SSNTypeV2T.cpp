// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/SP/SP_Ph1_SSNTypeV2T.h>

using namespace CPS;

SP::Ph1::SSNTypeV2T::SSNTypeV2T(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel) {
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  mParametersSet = false;
}

void SP::Ph1::SSNTypeV2T::setParameters(const MatrixComp &A,
                                        const MatrixComp &B,
                                        const MatrixComp &C,
                                        const MatrixComp &D) {

  if (A.cols() != A.rows())
    throw std::invalid_argument("A needs to be square.");

  if (B.rows() != A.cols())
    throw std::invalid_argument(
        "Rows of B do not match columns and rows of A.");

  if (B.cols() != 1)
    throw std::invalid_argument(
        "Two terminal V-type SSN component only has one external voltage, so "
        "column number of B must be one!");

  if (C.rows() != 1)
    throw std::invalid_argument(
        "A two terminal V type SSN component only has one output current. C "
        "must have exactly one row.");

  if (C.cols() != A.rows())
    throw std::invalid_argument("Columns of C do not match rows of A.");

  if (D.rows() != 1)
    throw std::invalid_argument(
        "A two terminal V type SSN component only has one output current. D "
        "must have exactly one row.");

  if (D.cols() != B.cols())
    throw std::invalid_argument(
        "Columns of D do not match columns of B and rows of input vector u, "
        "which need to be one.");

  mA = A;
  mB = B;
  mC = C;
  mD = D;

  mX = Matrix::Zero(mA.rows(), 1);
  mU = Matrix::Zero(mB.cols(), 1);
  mUPrev = Matrix::Zero(mB.cols(), 1);

  mParametersSet = true;
}

SimPowerComp<Complex>::Ptr SP::Ph1::SSNTypeV2T::clone(String name) {
  auto copy = SSNTypeV2T::make(name, mLogLevel);
  copy->setParameters(mA, mB, mC, mD);
  return copy;
}

void SP::Ph1::SSNTypeV2T::calculateAdmittance(Real omega) {
  MatrixComp H_inv =
      omega * Complex(0, 1.) * Matrix::Identity(mA.rows(), mA.cols()) - mA;

  MatrixComp H = MatrixComp(H_inv.rows(), H_inv.cols());

  H = H_inv.inverse().eval();

  mAdmittance = (mC.eval() * H * mB.eval() + mD.eval()).value();

  if (!Math::isFinite(mAdmittance) || std::abs(mAdmittance) < DOUBLE_EPSILON)
    throw std::invalid_argument(
        "calculateAdmittance: Admittance is near zero or infinite.");
}

void SP::Ph1::SSNTypeV2T::initializeFromNodesAndTerminals(Real frequency) {

  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminals ---");
  if (!mParametersSet)
    throw std::logic_error("Not initialized.");

  Real omega = 2 * PI * frequency;

  calculateAdmittance(omega);

  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  **mIntfCurrent = mAdmittance * **mIntfVoltage;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nImpedance [Ohm]: {:s}"
                     "\nAdmittance [S]: {:s}",
                     Logger::complexToString(1. / mAdmittance),
                     Logger::complexToString(mAdmittance));
  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

void SP::Ph1::SSNTypeV2T::mnaCompInitialize(Real omega, Real timeStep,
                                            Attribute<Matrix>::Ptr leftVector) {

  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminals ---");
  if (!mParametersSet)
    throw std::logic_error("Not initialized.");

  updateMatrixNodeIndices();

  calculateAdmittance(omega);

  SPDLOG_LOGGER_INFO(mSLog, "\nImpedance [Ohm]: {:s}",
                     Logger::complexToString(1. / mAdmittance));
  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- MNA initialization ---"
                     "\nInitial voltage {:s}"
                     "\nInitial current {:s}"
                     "\n--- MNA initialization finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)));
}

void SP::Ph1::SSNTypeV2T::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {

  MNAStampUtils::stampAdmittance(mAdmittance, systemMatrix, matrixNodeIndex(0),
                                 matrixNodeIndex(1), terminalNotGrounded(0),
                                 terminalNotGrounded(1), mSLog);
}

void SP::Ph1::SSNTypeV2T::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::SSNTypeV2T::mnaCompPostStep(Real time, Int timeStepCount,
                                          Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void SP::Ph1::SSNTypeV2T::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void SP::Ph1::SSNTypeV2T::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mAdmittance * (**mIntfVoltage);
}
