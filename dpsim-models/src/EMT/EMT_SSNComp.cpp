// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_SSNComp.h>

using namespace CPS;

EMT::SSNComp::SSNComp(String uid, String name, Int inputSize, Int outputSize,
                      Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel), mTimeStep(0.0),
      mW(Matrix::Zero(outputSize, inputSize)),
      mYHist(Matrix::Zero(outputSize, 1)), mInputSize(inputSize),
      mOutputSize(outputSize), mX(mAttributes->create<Matrix>("x")) {
  mParametersSet = false;
}

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
}

void EMT::SSNComp::recomputeDiscreteModel() {
  Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, mTimeStep, mdA, mdB);
  mW = mC * mdB + mD;
}

void EMT::SSNComp::updateStateSpaceModel() {
  // For linear components, the default implementation does nothing.
}

void EMT::SSNComp::mnaCompInitialize(Real, Real timeStep,
                                     Attribute<Matrix>::Ptr) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before initialization.");

  mTimeStep = timeStep;
  updateMatrixNodeIndices();

  recomputeDiscreteModel();
  mYHist = calculateHistoryVector();
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
