// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph1_MixedVTypeVariableSSNComp.h>
#include <dpsim-models/MNAStampUtils.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

DP::Ph1::MixedVTypeVariableSSNComp::MixedVTypeVariableSSNComp(
    String uid, String name, Int realStateCount, Int complexStateCount,
    Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mParameterChanged(false), mRealStateCount(realStateCount),
      mComplexStateCount(complexStateCount), mTimeStep(0.0), mW(0.0, 0.0),
      mYHist(0.0, 0.0), mX(mAttributes->create<Matrix>("x")) {
  mPhaseType = PhaseType::Single;
  setTerminalNumber(2);

  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  mParametersSet = false;
}

Int DP::Ph1::MixedVTypeVariableSSNComp::stateSize() const {
  return mRealStateCount + 2 * mComplexStateCount;
}

Matrix DP::Ph1::MixedVTypeVariableSSNComp::packComplex(const Complex &c) {
  Matrix v(2, 1);
  v(0, 0) = c.real();
  v(1, 0) = c.imag();
  return v;
}

Complex DP::Ph1::MixedVTypeVariableSSNComp::unpackComplex(const Matrix &v) {
  return Complex(v(0, 0), v(1, 0));
}

Attribute<MatrixComp>::Ptr
DP::Ph1::MixedVTypeVariableSSNComp::inputAttribute() const {
  return mIntfVoltage;
}

Attribute<MatrixComp>::Ptr
DP::Ph1::MixedVTypeVariableSSNComp::outputAttribute() const {
  return mIntfCurrent;
}

MatrixComp
DP::Ph1::MixedVTypeVariableSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp vInit = MatrixComp::Zero(1, 1);
  vInit(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  return vInit;
}

void DP::Ph1::MixedVTypeVariableSSNComp::setParameters(const Matrix &A,
                                                       const Matrix &B,
                                                       const Matrix &C,
                                                       const Matrix &D) {
  setParameters(A, B, C, D, Matrix::Zero(A.rows(), 1), Matrix::Zero(2, 1));
}

void DP::Ph1::MixedVTypeVariableSSNComp::setParameters(
    const Matrix &A, const Matrix &B, const Matrix &C, const Matrix &D,
    const Matrix &E, const Matrix &F) {
  mParametersSet = false;

  const Matrix::Index n = stateSize();

  if (A.rows() != n || A.cols() != n)
    throw std::invalid_argument("A has invalid dimensions.");

  if (B.rows() != n || B.cols() != 2)
    throw std::invalid_argument("B has invalid dimensions.");

  if (C.rows() != 2 || C.cols() != n)
    throw std::invalid_argument("C has invalid dimensions.");

  if (D.rows() != 2 || D.cols() != 2)
    throw std::invalid_argument("D has invalid dimensions.");

  if (E.rows() != n || E.cols() != 1)
    throw std::invalid_argument("E has invalid dimensions.");

  if (F.rows() != 2 || F.cols() != 1)
    throw std::invalid_argument("F has invalid dimensions.");

  mA = A;
  mB = B;
  mC = C;
  mD = D;
  mE = E;
  mF = F;

  **mX = Matrix::Zero(n, 1);

  mdA = Matrix::Zero(n, n);
  mdB = Matrix::Zero(n, 2);
  mdE = Matrix::Zero(n, 1);

  mW = Complex(0.0, 0.0);
  mYHist = Complex(0.0, 0.0);

  mParametersSet = true;
}

const Matrix &DP::Ph1::MixedVTypeVariableSSNComp::stateOffset() const {
  return mE;
}

const Matrix &DP::Ph1::MixedVTypeVariableSSNComp::outputOffset() const {
  return mF;
}

void DP::Ph1::MixedVTypeVariableSSNComp::setStateOffset(const Matrix &E) {
  if (E.rows() != stateSize() || E.cols() != 1)
    throw std::invalid_argument("State offset vector has invalid dimensions.");

  mE = E;
}

void DP::Ph1::MixedVTypeVariableSSNComp::setOutputOffset(const Matrix &F) {
  if (F.rows() != 2 || F.cols() != 1)
    throw std::invalid_argument("Output offset vector has invalid dimensions.");

  mF = F;
}

Matrix DP::Ph1::MixedVTypeVariableSSNComp::calculateHistoryVectorReal() const {
  const Matrix u = packComplex((**inputAttribute())(0, 0));
  return mC * (mdA * (**mX) + mdB * u + mdE) + mF;
}

void DP::Ph1::MixedVTypeVariableSSNComp::updateState(const Complex &uOld,
                                                     const Complex &uNew) {
  const Matrix u2Old = packComplex(uOld);
  const Matrix u2New = packComplex(uNew);
  **mX = mdA * (**mX) + mdB * (u2New + u2Old) + mdE;
}

void DP::Ph1::MixedVTypeVariableSSNComp::recomputeDiscreteModel() {
  Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, mE, mTimeStep, mdA, mdB,
                                               mdE);

  // Fold the 2x2 real u->y block into a complex admittance; requires it to
  // have the [[a,-b],[b,a]] complex-multiplication structure.
  const Matrix wReal = mC * mdB + mD;
  mW = Complex(wReal(0, 0), wReal(1, 0));
}

void DP::Ph1::MixedVTypeVariableSSNComp::updateStateSpaceModel() {
  mParameterChanged = updateComponentParameters();

  if (mParameterChanged)
    recomputeDiscreteModel();
}

Bool DP::Ph1::MixedVTypeVariableSSNComp::hasParameterChanged() {
  return mParameterChanged;
}

void DP::Ph1::MixedVTypeVariableSSNComp::updateLogAttributes(
    const Matrix &) const {}

void DP::Ph1::MixedVTypeVariableSSNComp::initializeFromNodesAndTerminals(
    Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  if (mRealStateCount != 0)
    throw std::logic_error(
        "The default initializeFromNodesAndTerminals() only supports a "
        "purely complex-envelope state (realStateCount() == 0); components "
        "with real control states must override it.");

  const MatrixComp uInit = buildInitialInputFromNodes(frequency);
  const Complex u = uInit(0, 0);

  **mIntfVoltage = uInit;
  **mX = Matrix::Zero(stateSize(), 1);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from nodes and terminals ---");

  Bool converged = false;

  Matrix uColumn = Matrix::Zero(2, 1);
  uColumn(0, 0) = u.real();
  uColumn(1, 0) = u.imag();

  for (Int iter = 0; iter < mInitializationMaxIterations; ++iter) {
    const Matrix xPrev = **mX;

    updateComponentParameters();

    // mA is already carrier-shifted, so steady state is 0 = mA*x + mB*u + mE.
    **mX = -mA.inverse() * (mB * uColumn + mE);

    if ((**mX).isApprox(xPrev, mInitializationTolerance)) {
      converged = true;
      break;
    }
  }

  if (!converged) {
    SPDLOG_LOGGER_WARN(
        mSLog,
        "Fixed-point initialization did not converge within {} iterations.",
        mInitializationMaxIterations);
  }

  updateComponentParameters();

  const Matrix yReal = mC * (**mX) + mD * packComplex(u) + mF;
  (**mIntfCurrent)(0, 0) = unpackComplex(yReal);

  mParameterChanged = false;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nInput u: {:s}"
      "\nOutput y: {:s}"
      "\nState x: {:s}"
      "\n--- Initialization from nodes and terminals finished ---",
      Logger::matrixCompToString(**mIntfVoltage),
      Logger::matrixCompToString(**mIntfCurrent), Logger::matrixToString(**mX));
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompInitialize(
    Real, Real timeStep, Attribute<Matrix>::Ptr) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before initialization.");

  mTimeStep = timeStep;
  updateMatrixNodeIndices();

  updateComponentParameters();
  recomputeDiscreteModel();
  mYHist = unpackComplex(calculateHistoryVectorReal());
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mX);
  prevStepDependencies.push_back(inputAttribute());
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompPreStep(Real, Int) {
  updateStateSpaceModel();
  mYHist = unpackComplex(calculateHistoryVectorReal());
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(inputAttribute());
  modifiedAttributes.push_back(outputAttribute());
  modifiedAttributes.push_back(mX);
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    MNAStampUtils::stampAdmittance(
        mW, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
        terminalNotGrounded(0), terminalNotGrounded(1), mSLog, mNumFreqs, freq);
  }
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    if (terminalNotGrounded(0))
      Math::setVectorElement(rightVector, matrixNodeIndex(0), mYHist, mNumFreqs,
                             freq);
    if (terminalNotGrounded(1))
      Math::setVectorElement(rightVector, matrixNodeIndex(1), -mYHist,
                             mNumFreqs, freq);
  }
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  for (UInt freq = 0; freq < mNumFreqs; freq++) {
    (**mIntfVoltage)(0, freq) = 0;
    if (terminalNotGrounded(1))
      (**mIntfVoltage)(0, freq) = Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(1), mNumFreqs, freq);
    if (terminalNotGrounded(0))
      (**mIntfVoltage)(0, freq) -= Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(0), mNumFreqs, freq);
  }
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  (**mIntfCurrent)(0, 0) = mW * (**mIntfVoltage)(0, 0) + mYHist;
}

void DP::Ph1::MixedVTypeVariableSSNComp::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  const Complex uOld = (**mIntfVoltage)(0, 0);
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  updateState(uOld, (**mIntfVoltage)(0, 0));
  updateLogAttributes(packComplex((**mIntfVoltage)(0, 0)));
}
