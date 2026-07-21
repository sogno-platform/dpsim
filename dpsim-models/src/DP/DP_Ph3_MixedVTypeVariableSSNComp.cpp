// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_MixedVTypeVariableSSNComp.h>
#include <dpsim-models/MNAStampUtils.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

DP::Ph3::MixedVTypeVariableSSNComp::MixedVTypeVariableSSNComp(
    String uid, String name, Int realStateCount, Int complexStateCount,
    Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mParameterChanged(false), mRealStateCount(realStateCount),
      mComplexStateCount(complexStateCount), mTimeStep(0.0),
      mW(MatrixComp::Zero(mPhaseCount, mPhaseCount)),
      mYHist(MatrixComp::Zero(mPhaseCount, 1)),
      mX(mAttributes->create<Matrix>("x")) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);

  **mIntfVoltage = MatrixComp::Zero(mPhaseCount, 1);
  **mIntfCurrent = MatrixComp::Zero(mPhaseCount, 1);

  mParametersSet = false;
}

Int DP::Ph3::MixedVTypeVariableSSNComp::stateSize() const {
  return mRealStateCount + 2 * mComplexStateCount;
}

UInt DP::Ph3::MixedVTypeVariableSSNComp::getStateCount() const {
  return static_cast<UInt>(stateSize());
}

const Matrix &DP::Ph3::MixedVTypeVariableSSNComp::getDiscreteA() const {
  return mdA;
}

const Matrix &DP::Ph3::MixedVTypeVariableSSNComp::getDiscreteB() const {
  return mdB;
}

const Matrix &DP::Ph3::MixedVTypeVariableSSNComp::getC() const { return mC; }

Matrix DP::Ph3::MixedVTypeVariableSSNComp::packComplex(const MatrixComp &c) {
  const Matrix::Index m = c.rows();
  Matrix v(2 * m, 1);
  for (Matrix::Index i = 0; i < m; ++i) {
    v(2 * i, 0) = c(i, 0).real();
    v(2 * i + 1, 0) = c(i, 0).imag();
  }
  return v;
}

MatrixComp DP::Ph3::MixedVTypeVariableSSNComp::unpackComplex(const Matrix &v) {
  const Matrix::Index m = v.rows() / 2;
  MatrixComp c(m, 1);
  for (Matrix::Index i = 0; i < m; ++i)
    c(i, 0) = Complex(v(2 * i, 0), v(2 * i + 1, 0));
  return c;
}

MatrixComp
DP::Ph3::MixedVTypeVariableSSNComp::foldComplexMatrix(const Matrix &real) {
  const Matrix::Index m = real.rows() / 2;
  MatrixComp c(m, real.cols() / 2);
  for (Matrix::Index i = 0; i < m; ++i)
    for (Matrix::Index j = 0; j < real.cols() / 2; ++j)
      // Trust the [[a,-b],[b,a]] complex-multiplication structure of each
      // 2x2 block (Irc = (U - Vc)/Rc is an exact real-scalar per-phase
      // relation), as DP::SSNComp does; not re-checked at runtime.
      c(i, j) = Complex(real(2 * i, 2 * j), real(2 * i + 1, 2 * j));
  return c;
}

Attribute<MatrixComp>::Ptr
DP::Ph3::MixedVTypeVariableSSNComp::inputAttribute() const {
  return mIntfVoltage;
}

Attribute<MatrixComp>::Ptr
DP::Ph3::MixedVTypeVariableSSNComp::outputAttribute() const {
  return mIntfCurrent;
}

MatrixComp
DP::Ph3::MixedVTypeVariableSSNComp::buildInitialInputFromNodes(Real) {
  MatrixComp vInit = MatrixComp::Zero(mPhaseCount, 1);
  vInit(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  vInit(1, 0) = vInit(0, 0) * SHIFT_TO_PHASE_B;
  vInit(2, 0) = vInit(0, 0) * SHIFT_TO_PHASE_C;
  return vInit;
}

void DP::Ph3::MixedVTypeVariableSSNComp::setParameters(const Matrix &A,
                                                       const Matrix &B,
                                                       const Matrix &C,
                                                       const Matrix &D) {
  setParameters(A, B, C, D, Matrix::Zero(A.rows(), 1),
                Matrix::Zero(2 * mPhaseCount, 1));
}

void DP::Ph3::MixedVTypeVariableSSNComp::setParameters(
    const Matrix &A, const Matrix &B, const Matrix &C, const Matrix &D,
    const Matrix &E, const Matrix &F) {
  mParametersSet = false;

  const Matrix::Index n = stateSize();
  const Matrix::Index m = 2 * mPhaseCount;

  if (A.rows() != n || A.cols() != n)
    throw std::invalid_argument("A has invalid dimensions.");

  if (B.rows() != n || B.cols() != m)
    throw std::invalid_argument("B has invalid dimensions.");

  if (C.rows() != m || C.cols() != n)
    throw std::invalid_argument("C has invalid dimensions.");

  if (D.rows() != m || D.cols() != m)
    throw std::invalid_argument("D has invalid dimensions.");

  if (E.rows() != n || E.cols() != 1)
    throw std::invalid_argument("E has invalid dimensions.");

  if (F.rows() != m || F.cols() != 1)
    throw std::invalid_argument("F has invalid dimensions.");

  mA = A;
  mB = B;
  mC = C;
  mD = D;
  mE = E;
  mF = F;

  **mX = Matrix::Zero(n, 1);

  mdA = Matrix::Zero(n, n);
  mdB = Matrix::Zero(n, m);
  mdE = Matrix::Zero(n, 1);

  mW = MatrixComp::Zero(mPhaseCount, mPhaseCount);
  mYHist = MatrixComp::Zero(mPhaseCount, 1);

  mParametersSet = true;
}

const Matrix &DP::Ph3::MixedVTypeVariableSSNComp::stateOffset() const {
  return mE;
}

const Matrix &DP::Ph3::MixedVTypeVariableSSNComp::outputOffset() const {
  return mF;
}

void DP::Ph3::MixedVTypeVariableSSNComp::setStateOffset(const Matrix &E) {
  if (E.rows() != stateSize() || E.cols() != 1)
    throw std::invalid_argument("State offset vector has invalid dimensions.");

  mE = E;
}

void DP::Ph3::MixedVTypeVariableSSNComp::setOutputOffset(const Matrix &F) {
  if (F.rows() != 2 * mPhaseCount || F.cols() != 1)
    throw std::invalid_argument("Output offset vector has invalid dimensions.");

  mF = F;
}

Matrix DP::Ph3::MixedVTypeVariableSSNComp::calculateHistoryVectorReal() const {
  const Matrix u = packComplex(**inputAttribute());
  return mC * (mdA * (**mX) + mdB * u + mdE) + mF;
}

void DP::Ph3::MixedVTypeVariableSSNComp::updateState(const MatrixComp &uOld,
                                                     const MatrixComp &uNew) {
  const Matrix uOldReal = packComplex(uOld);
  const Matrix uNewReal = packComplex(uNew);
  **mX = mdA * (**mX) + mdB * (uNewReal + uOldReal) + mdE;
}

void DP::Ph3::MixedVTypeVariableSSNComp::recomputeDiscreteModel() {
  Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, mE, mTimeStep, mdA, mdB,
                                               mdE);

  // Fold the 2m x 2m real u->y block into a 3x3 complex admittance; each 2x2
  // block is required to have the [[a,-b],[b,a]] complex-multiplication form.
  mW = foldComplexMatrix(mC * mdB + mD);
}

void DP::Ph3::MixedVTypeVariableSSNComp::updateStateSpaceModel() {
  mParameterChanged = updateComponentParameters();

  if (mParameterChanged)
    recomputeDiscreteModel();
}

Bool DP::Ph3::MixedVTypeVariableSSNComp::hasParameterChanged() {
  return mParameterChanged;
}

void DP::Ph3::MixedVTypeVariableSSNComp::updateLogAttributes(
    const Matrix &) const {}

void DP::Ph3::MixedVTypeVariableSSNComp::initializeFromNodesAndTerminals(
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

  **mIntfVoltage = uInit;
  **mX = Matrix::Zero(stateSize(), 1);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from nodes and terminals ---");

  Bool converged = false;

  const Matrix uColumn = packComplex(uInit);

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

  const Matrix yReal = mC * (**mX) + mD * uColumn + mF;
  **mIntfCurrent = unpackComplex(yReal);

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

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompInitialize(
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

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mX);
  prevStepDependencies.push_back(inputAttribute());
}

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompPreStep(Real, Int) {
  updateStateSpaceModel();
  mYHist = unpackComplex(calculateHistoryVectorReal());
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(inputAttribute());
  modifiedAttributes.push_back(outputAttribute());
  modifiedAttributes.push_back(mX);
}

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittanceMatrix(
      mW, systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1),
      terminalNotGrounded(0), terminalNotGrounded(1), mSLog);
}

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < mPhaseCount; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             mYHist(phase, 0));
  }
  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < mPhaseCount; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(1, phase),
                             -mYHist(phase, 0));
  }
}

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  **mIntfVoltage = MatrixComp::Zero(mPhaseCount, 1);

  if (terminalNotGrounded(1)) {
    for (Int phase = 0; phase < mPhaseCount; ++phase)
      (**mIntfVoltage)(phase, 0) =
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, phase));
  }
  if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < mPhaseCount; ++phase)
      (**mIntfVoltage)(phase, 0) -=
          Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, phase));
  }
}

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  **mIntfCurrent = mW * (**mIntfVoltage) + mYHist;
}

void DP::Ph3::MixedVTypeVariableSSNComp::mnaCompPostStep(
    Real, Int, Attribute<Matrix>::Ptr &leftVector) {
  const MatrixComp uOld = **mIntfVoltage;
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  updateState(uOld, **mIntfVoltage);
  updateLogAttributes(packComplex(**mIntfVoltage));
}
