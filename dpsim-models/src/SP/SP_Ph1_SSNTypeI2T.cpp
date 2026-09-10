// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/SP/SP_Ph1_SSNTypeI2T.h>

using namespace CPS;

SP::Ph1::SSNTypeI2T::SSNTypeI2T(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel) {
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  mParametersSet = false;
}

void SP::Ph1::SSNTypeI2T::setParameters(const MatrixComp &A,
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
        "Two terminal I-type SSN component only has one "
        "external current, so column number of B must be one.");

  if (C.rows() != 1)
    throw std::invalid_argument(
        "A two terminal I type SSN component only has"
        "one output voltage. C must have exactly one row.");

  if (C.cols() != A.rows())
    throw std::invalid_argument("Columns of C do not match rows of A.");

  if (D.rows() != 1)
    throw std::invalid_argument(
        "A two terminal I type SSN component only has"
        "one output voltage. D must have exactly one row.");

  if (D.cols() != B.cols())
    throw std::invalid_argument(
        "Columns of D do not match columns of B and rows"
        "of input vector u, which need to be one.");

  mA = A;
  mB = B;
  mC = C;
  mD = D;

  mX = Matrix::Zero(mA.rows(), 1);
  mU = Matrix::Zero(mB.cols(), 1);
  mUPrev = Matrix::Zero(mB.cols(), 1);

  mParametersSet = true;
}

SimPowerComp<Complex>::Ptr SP::Ph1::SSNTypeI2T::clone(String name) {
  auto copy = SSNTypeI2T::make(name, mLogLevel);
  copy->setParameters(mA, mB, mC, mD);
  return copy;
}

Complex SP::Ph1::SSNTypeI2T::computeAdmittance(Real omega) const {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "the admittance can be calculated.");

  // I-type: y = V, u = I, so the transfer is an impedance and has to be inverted.
  Complex impedance = Math::steadyStateTransfer(mA, mB, mC, mD, omega).value();

  if (std::abs(impedance) < DOUBLE_EPSILON)
    throw std::invalid_argument("computeAdmittance: impedance is near zero.");

  return 1. / impedance;
}

Complex SP::Ph1::SSNTypeI2T::steadyStateAdmittance(Real frequency) const {
  return computeAdmittance(2. * PI * frequency);
}

void SP::Ph1::SSNTypeI2T::initializeFromNodesAndTerminals(Real frequency) {

  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminals ---");
  if (!mParametersSet)
    throw std::logic_error("Not initialized.");

  mAdmittance = steadyStateAdmittance(frequency);

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

void SP::Ph1::SSNTypeI2T::mnaCompInitialize(Real omega, Real timeStep,
                                            Attribute<Matrix>::Ptr leftVector) {

  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminals ---");
  if (!mParametersSet)
    throw std::logic_error("Not initialized.");

  updateMatrixNodeIndices();

  mAdmittance = computeAdmittance(omega);

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

void SP::Ph1::SSNTypeI2T::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {

  MNAStampUtils::stampAdmittance(mAdmittance, systemMatrix, matrixNodeIndex(0),
                                 matrixNodeIndex(1), terminalNotGrounded(0),
                                 terminalNotGrounded(1), mSLog);
}

void SP::Ph1::SSNTypeI2T::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::SSNTypeI2T::mnaCompPostStep(Real time, Int timeStepCount,
                                          Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void SP::Ph1::SSNTypeI2T::mnaCompUpdateVoltage(const Matrix &leftVector) {
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

void SP::Ph1::SSNTypeI2T::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mAdmittance * (**mIntfVoltage);
}
