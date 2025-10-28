/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_SSNTypeV2T.h>

using namespace CPS;

EMT::Ph1::SSNTypeV2T::SSNTypeV2T(String uid, String name,
                                 Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mA(mAttributes->create<Matrix>("A")),
      mB(mAttributes->create<Matrix>("B")),
      mC(mAttributes->create<Matrix>("C")),
      mD(mAttributes->create<Matrix>("D")),
      mdA(mAttributes->create<Matrix>("dA")),
      mdB(mAttributes->create<Matrix>("dB")),
      mdC(mAttributes->create<Matrix>("dC")) {
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  mParametersSet = false;
}

void EMT::Ph1::SSNTypeV2T::setParameters(const Matrix A, const Matrix B,
                                         const Matrix C, const Matrix D) {
  try {
    if (A.cols() != A.rows())
      throw std::invalid_argument(
          "A needs to be quadratic; Set all matrices to scalar zero.");
    **mA = A;
    mX = Matrix::Zero((**mA).rows(), 1);
  } catch (std::exception &e) {
    SPDLOG_LOGGER_ERROR(mSLog, "Component {} : {}\n", **mName, e.what());
    setSSNMatricesToZero();
    return;
  }
  try {
    if (B.rows() != (**mA).cols())
      throw std::invalid_argument(
          "Invalid dimensions; rows of B do not match columns and rows of A! "
          "Set all matrices to scalar zero.");
    if (B.cols() != 1)
      throw std::invalid_argument(
          "Invalid dimensions; two terminal V-type SSN component only has on "
          "external voltage, so column number of B must be one! Set all "
          "matrices to scalar zero.");
    **mB = B;
    mU = Matrix::Zero((**mB).cols(), 1);
    mUOld = Matrix::Zero((**mB).cols(), 1);
  } catch (std::exception &e) {
    SPDLOG_LOGGER_ERROR(mSLog, "Component {} : {}\n", **mName, e.what());
    setSSNMatricesToZero();
    return;
  }
  try {
    if (C.rows() != 1)
      throw std::invalid_argument(
          "Invalid dimensions; a two terminal V type SSN component only has "
          "one output current! C must have exactly one row! Set all matrices "
          "to scalar zero.");
    if (C.cols() != (**mA).rows())
      throw std::invalid_argument(
          "Invalid dimensions; columns of C do not match rows of A! Set all "
          "matrices to scalar zero.");
    **mC = C;
  } catch (std::exception &e) {
    SPDLOG_LOGGER_ERROR(mSLog, "Component {} : {}\n", **mName, e.what());
    setSSNMatricesToZero();
    return;
  }
  try {
    if (D.rows() != 1)
      throw std::invalid_argument(
          "Invalid dimensions; a two terminal V type SSN component only has "
          "one output current! D must have exactly one row! Set all matrices "
          "to scalar zero.");
    if (D.cols() != (**mB).cols())
      throw std::invalid_argument(
          "Invalid dimensions; columns of D do not match columns of B and rows "
          "of input vector u, which need to be one! Set all matrices to scalar "
          "zero.");
    **mD = D;
  } catch (std::exception &e) {
    SPDLOG_LOGGER_ERROR(mSLog, "Component {} : {}\n", **mName, e.what());
    setSSNMatricesToZero();
    return;
  }
  mParametersSet = true;
}

SimPowerComp<Real>::Ptr EMT::Ph1::SSNTypeV2T::clone(String name) {
  auto copy = SSNTypeV2T::make(name, mLogLevel);
  copy->setParameters(**mA, **mB, **mC, **mD);
  return copy;
}

void EMT::Ph1::SSNTypeV2T::initializeFromNodesAndTerminals(Real frequency) {

  (**mIntfCurrent)(0, 0) = 0;
  (**mIntfVoltage)(0, 0) = 0;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:f}"
                     "\nCurrent: {:f}"
                     "\nTerminal 0 voltage: {:f}"
                     "\nTerminal 1 voltage: {:f}"
                     "\n--- Initialization from powerflow finished ---",
                     (**mIntfVoltage)(0, 0), (**mIntfCurrent)(0, 0),
                     (RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)).real(),
                     (RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)).real());
}

void EMT::Ph1::SSNTypeV2T::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();

  Math::calculateStateSpaceTrapezoidalMatrices(**mA, **mB, timeStep, **mdA,
                                               **mdB);
  mW = Matrix::Zero(1, 1);
  mW = (**mC) * (**mdB) + (**mD);
}

void EMT::Ph1::SSNTypeV2T::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {

  MNAStampUtils::stampConductance(mW(0, 0), systemMatrix, matrixNodeIndex(0),
                                  matrixNodeIndex(1), terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
}

void EMT::Ph1::SSNTypeV2T::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  // Update internal state
  mYHist = (**mC) * ((**mdA) * mX + (**mdB) * mU);
  if (terminalNotGrounded(0))
    Math::setVectorElement(rightVector, matrixNodeIndex(0), mYHist(0, 0));
  if (terminalNotGrounded(1))
    Math::setVectorElement(rightVector, matrixNodeIndex(1), -mYHist(0, 0));
}

void EMT::Ph1::SSNTypeV2T::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {

  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
}

void EMT::Ph1::SSNTypeV2T::mnaCompPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::SSNTypeV2T::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::SSNTypeV2T::mnaCompPostStep(Real time, Int timeStepCount,
                                           Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  ssnUpdateState();
}

void EMT::Ph1::SSNTypeV2T::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  mUOld = mU;
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
  mU = **mIntfVoltage;
}

void EMT::Ph1::SSNTypeV2T::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mW * (**mIntfVoltage) + mYHist;
}

void CPS::EMT::Ph1::SSNTypeV2T::setSSNMatricesToZero() {
  **mA = Matrix::Zero(1, 1);
  **mB = Matrix::Zero(1, 1);
  **mC = Matrix::Zero(1, 1);
  **mD = Matrix::Zero(1, 1);

  **mdA = Matrix::Zero(1, 1);
  **mdB = Matrix::Zero(1, 1);
  **mdC = Matrix::Zero(1, 1);

  mX = Matrix::Zero(1, 1);
  mU = Matrix::Zero(1, 1);
  mUOld = Matrix::Zero(1, 1);
}

void EMT::Ph1::SSNTypeV2T::ssnUpdateState() {
  mX = (**mdA) * mX + (**mdB) * (mU + mUOld);
}

void EMT::Ph1::SSNTypeV2T::manualInit(Matrix initialState, Matrix initialInput,
                                      Matrix initialOldInput, Real initCurrent,
                                      Real initVoltage) {
  try {
    if (mParametersSet == false)
      throw std::invalid_argument(
          "Set parameters first! Setting x, u, u_old to zero.");
    if (initialState.cols() != 1)
      throw std::invalid_argument("State matrix is a vector; column number "
                                  "must be one. Setting x, u, u_old to zero.");
    if (initialState.rows() != (**mA).cols())
      throw std::invalid_argument(
          "State row number does not match row and column number of A. Setting "
          "x, u, u_old to zero.");
    if (initialInput.cols() != 1 || initialOldInput.cols() != 1)
      throw std::invalid_argument("Input matrices are vectors; column number "
                                  "must be one. Setting x, u, u_old to zero.");
    if (initialInput.rows() != (**mB).cols() ||
        initialOldInput.rows() != (**mB).cols())
      throw std::invalid_argument("Input vector rows have to match columns of "
                                  "Matrix B; Setting x, u, u_old to zero.");
    mX = initialState;
    mU = initialInput;
    mUOld = initialOldInput;
    (**mIntfCurrent)(0, 0) = initCurrent;
    (**mIntfVoltage)(0, 0) = initVoltage;
  } catch (std::exception &e) {
    SPDLOG_LOGGER_ERROR(mSLog, "Component {} : {}\n", **mName, e.what());
    (**mIntfCurrent)(0, 0) = 0;
    (**mIntfVoltage)(0, 0) = 0;
  }
}