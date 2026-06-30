// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_SSNComp.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

DP::SSNComp::SSNComp(String uid, String name, Int inputSize, Int outputSize,
                     Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel), mTimeStep(0.0),
      mW(MatrixComp::Zero(outputSize, inputSize)),
      mYHist(MatrixComp::Zero(outputSize, 1)), mInputSize(inputSize),
      mOutputSize(outputSize), mX(mAttributes->create<MatrixComp>("x")) {
  mParametersSet = false;
}

UInt DP::SSNComp::getStateCount() const { return static_cast<UInt>(mA.rows()); }

const MatrixComp &DP::SSNComp::getDiscreteA() const { return mDiscreteA; }

const MatrixComp &DP::SSNComp::getDiscreteB() const { return mDiscreteB; }

const Matrix &DP::SSNComp::getC() const { return mC; }

void DP::SSNComp::setParameters(const Matrix &A, const Matrix &B,
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

  mA = A;
  mB = B;
  mC = C;
  mD = D;

  **mX = MatrixComp::Zero(mA.rows(), 1);

  mDiscreteA = MatrixComp::Zero(mA.rows(), mA.cols());
  mDiscreteB = MatrixComp::Zero(mB.rows(), mB.cols());

  mW = MatrixComp::Zero(mOutputSize, mInputSize);
  mYHist = MatrixComp::Zero(mOutputSize, 1);

  mParametersSet = true;
}

Matrix DP::SSNComp::buildAugmentedA(Real omega) const {
  const Matrix::Index n = mA.rows();
  const Matrix wI = omega * Matrix::Identity(n, n);

  Matrix aAug = Matrix::Zero(2 * n, 2 * n);
  aAug.topLeftCorner(n, n) = mA;
  aAug.topRightCorner(n, n) = wI;
  aAug.bottomLeftCorner(n, n) = -wI;
  aAug.bottomRightCorner(n, n) = mA;
  return aAug;
}

Matrix DP::SSNComp::buildAugmentedB() const {
  const Matrix::Index n = mA.rows();

  Matrix bAug = Matrix::Zero(2 * n, 2 * mInputSize);
  bAug.topLeftCorner(n, mInputSize) = mB;
  bAug.bottomRightCorner(n, mInputSize) = mB;
  return bAug;
}

void DP::SSNComp::recomputeDiscreteModel(Real omega) {
  const Matrix::Index n = mA.rows();

  // Discretize the real-augmented model with the same helper as EMT-SSN.
  const Matrix aAug = buildAugmentedA(omega);
  const Matrix bAug = buildAugmentedB();
  Matrix dAaug = Matrix::Zero(2 * n, 2 * n);
  Matrix dBaug = Matrix::Zero(2 * n, 2 * mInputSize);
  Math::calculateStateSpaceTrapezoidalMatrices(aAug, bAug, mTimeStep, dAaug,
                                               dBaug);

  // Fold the [[P, -Q], [Q, P]] rotation blocks back into complex P + jQ.
  mDiscreteA = dAaug.topLeftCorner(n, n).cast<Complex>() +
               Complex(0., 1.) * dAaug.bottomLeftCorner(n, n).cast<Complex>();
  mDiscreteB =
      dBaug.block(0, 0, n, mInputSize).cast<Complex>() +
      Complex(0., 1.) * dBaug.block(n, 0, n, mInputSize).cast<Complex>();

  mW = mC.cast<Complex>() * mDiscreteB + mD.cast<Complex>();
}

MatrixComp DP::SSNComp::calculateHistoryVector() const {
  return mC.cast<Complex>() *
         (mDiscreteA * (**mX) + mDiscreteB * (**inputAttribute()));
}

MatrixComp DP::SSNComp::calculateSteadyStateStateFromInput(const MatrixComp &u,
                                                           Real omega) const {
  MatrixComp h =
      Complex(0., omega) * MatrixComp::Identity(mA.rows(), mA.cols()) -
      mA.cast<Complex>();

  return h.inverse() * mB.cast<Complex>() * u;
}

MatrixComp
DP::SSNComp::calculateSteadyStateOutputFromInput(const MatrixComp &x,
                                                 const MatrixComp &u) const {
  return mC.cast<Complex>() * x + mD.cast<Complex>() * u;
}

void DP::SSNComp::updateState(const MatrixComp &uOld, const MatrixComp &uNew) {
  **mX = mDiscreteA * (**mX) + mDiscreteB * (uNew + uOld);
}

void DP::SSNComp::updateStateSpaceModel() {
  // For linear components, the default implementation does nothing.
}

void DP::SSNComp::mnaCompInitialize(Real omega, Real timeStep,
                                    Attribute<Matrix>::Ptr) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before initialization.");

  if (mNumFreqs != 1)
    throw std::logic_error(
        "DP SSN components currently support a single carrier frequency.");

  mTimeStep = timeStep;
  updateMatrixNodeIndices();

  recomputeDiscreteModel(omega);
  mYHist = calculateHistoryVector();
}

void DP::SSNComp::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
  prevStepDependencies.push_back(mX);
  prevStepDependencies.push_back(inputAttribute());
}

void DP::SSNComp::mnaCompPreStep(Real time, Int timeStepCount) {
  updateStateSpaceModel();
  mYHist = calculateHistoryVector();
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::SSNComp::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(inputAttribute());
  modifiedAttributes.push_back(outputAttribute());
  modifiedAttributes.push_back(mX);
}
