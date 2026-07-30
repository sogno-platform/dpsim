// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_SSNComp.h>

using namespace CPS;

namespace {
void requireFinite(const Matrix &matrix, const char *name) {
  if (!matrix.allFinite())
    throw std::invalid_argument(String(name) + " must contain finite values.");
}
} // namespace

EMT::SSNComp::SSNComp(String uid, String name, Int inputSize, Int outputSize,
                      Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel), mTimeStep(0.0),
      mW(Matrix::Zero(outputSize, inputSize)),
      mYHist(Matrix::Zero(outputSize, 1)), mInputSize(inputSize),
      mOutputSize(outputSize), mX(mAttributes->create<Matrix>("x")) {
  mParametersSet = false;
}

UInt EMT::SSNComp::getStateCount() const {
  return static_cast<UInt>(mA.rows());
}

std::vector<String> EMT::SSNComp::getLocalStateNames() const { return {}; }

std::vector<EMT::SSNComp::LocalAbcStateBlock>
EMT::SSNComp::getLocalAbcStateBlocks() const {
  // Default: no abc-frame state metadata. Derived components should override
  // this only for states known to form physical abc triples.
  return {};
}

const Matrix &EMT::SSNComp::getDiscreteA() const { return mdA; }

const Matrix &EMT::SSNComp::getDiscreteB() const { return mdB; }

const Matrix &EMT::SSNComp::getC() const { return mC; }

void EMT::SSNComp::setParameters(const Matrix &A, const Matrix &B,
                                 const Matrix &C, const Matrix &D) {
  mParametersSet = false;

  if (A.rows() != A.cols())
    throw std::invalid_argument("A must be square.");

  if (B.rows() != A.rows() || B.cols() != mInputSize)
    throw std::invalid_argument("B has invalid dimensions.");

  if (C.rows() != mOutputSize || C.cols() != A.rows())
    throw std::invalid_argument("C has invalid dimensions.");

  if (D.rows() != mOutputSize || D.cols() != mInputSize)
    throw std::invalid_argument("D has invalid dimensions.");

  requireFinite(A, "A");
  requireFinite(B, "B");
  requireFinite(C, "C");
  requireFinite(D, "D");

  mA = A;
  mB = B;
  mC = C;
  mD = D;

  **mX = Matrix::Zero(mA.rows(), 1);

  mdA = Matrix::Zero(mA.rows(), mA.cols());
  mdB = Matrix::Zero(mB.rows(), mB.cols());

  mW = Matrix::Zero(mOutputSize, mInputSize);
  mYHist = Matrix::Zero(mOutputSize, 1);

  mParametersSet = true;
}

Matrix EMT::SSNComp::calculateHistoryVector() const {
  return mC * (mdA * (**mX) + mdB * (**inputAttribute()));
}

MatrixComp
EMT::SSNComp::calculateSteadyStateStateFromInput(const MatrixComp &u,
                                                 Real frequency) const {
  if (mA.rows() == 0)
    return MatrixComp::Zero(0, 1);

  const Real omega = 2.0 * PI * frequency;
  MatrixComp h =
      Complex(0.0, omega) * MatrixComp::Identity(mA.rows(), mA.cols()) -
      mA.cast<Complex>();

  return h.inverse() * mB.cast<Complex>() * u;
}

MatrixComp
EMT::SSNComp::calculateSteadyStateOutputFromInput(const MatrixComp &x,
                                                  const MatrixComp &u) const {
  return mC.cast<Complex>() * x + mD.cast<Complex>() * u;
}

void EMT::SSNComp::updateState(const Matrix &uOld, const Matrix &uNew) {
  **mX = mdA * (**mX) + mdB * (uNew + uOld);
  if (!(**mX).allFinite())
    throw std::runtime_error("SSN state update produced a non-finite value.");
}

void EMT::SSNComp::updateLogAttributes(const Matrix &) const {
  // the default implementation does nothing.
}

void EMT::SSNComp::recomputeDiscreteModel() {
  Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, mTimeStep, mdA, mdB);
  mW = mC * mdB + mD;
  if (!mdA.allFinite() || !mdB.allFinite() || !mW.allFinite())
    throw std::runtime_error(
        "SSN trapezoidal discretization produced a non-finite value.");
}

void EMT::SSNComp::updateStateSpaceModel() {
  // For linear components, the default implementation does nothing.
}

void EMT::SSNComp::mnaCompInitialize(Real, Real timeStep,
                                     Attribute<Matrix>::Ptr) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before initialization.");

  if (!Math::isFinite(timeStep) || timeStep <= 0.0)
    throw std::invalid_argument("SSN time step must be finite and positive.");

  mTimeStep = timeStep;
  updateMatrixNodeIndices();

  recomputeDiscreteModel();
  mYHist = calculateHistoryVector();
  if (!mYHist.allFinite())
    throw std::runtime_error(
        "SSN initialization produced a non-finite history vector.");
}

void EMT::SSNComp::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mX);
  prevStepDependencies.push_back(inputAttribute());
}

void EMT::SSNComp::mnaCompPreStep(Real time, Int timeStepCount) {
  updateStateSpaceModel();
  mYHist = calculateHistoryVector();
  if (!mYHist.allFinite())
    throw std::runtime_error("SSN history vector contains a non-finite value.");
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::SSNComp::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(inputAttribute());
  modifiedAttributes.push_back(outputAttribute());
  modifiedAttributes.push_back(mX);
}
