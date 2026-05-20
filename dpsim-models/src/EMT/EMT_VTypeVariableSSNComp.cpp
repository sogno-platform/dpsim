// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_VTypeVariableSSNComp.h>

using namespace CPS;

EMT::VTypeVariableSSNComp::VTypeVariableSSNComp(String uid, String name,
                                                Int inputSize, Int outputSize,
                                                Logger::Level logLevel)
    : VTypeSSNComp(uid, name, inputSize, outputSize, logLevel),
      mParameterChanged(false), mE(Matrix::Zero(0, 1)), mdE(Matrix::Zero(0, 1)),
      mF(Matrix::Zero(outputSize, 1)) {}

Matrix EMT::VTypeVariableSSNComp::calculateHistoryVector() const {
  return mC * (mdA * (**mX) + mdB * (**inputAttribute()) + mdE) + mF;
}

MatrixComp EMT::VTypeVariableSSNComp::calculateSteadyStateStateFromInput(
    const MatrixComp &u, Real frequency) const {
  const Real omega = 2.0 * PI * frequency;
  MatrixComp h =
      Complex(0.0, omega) * MatrixComp::Identity(mA.rows(), mA.cols()) -
      mA.cast<Complex>();

  return h.inverse() * (mB.cast<Complex>() * u + mE.cast<Complex>());
}

MatrixComp EMT::VTypeVariableSSNComp::calculateSteadyStateOutputFromInput(
    const MatrixComp &x, const MatrixComp &u) const {
  return mC.cast<Complex>() * x + mD.cast<Complex>() * u + mF.cast<Complex>();
}

void EMT::VTypeVariableSSNComp::updateState(const Matrix &uOld,
                                            const Matrix &uNew) {
  **mX = mdA * (**mX) + mdB * (uNew + uOld) + mdE;
}

void EMT::VTypeVariableSSNComp::recomputeDiscreteModel() {
  Math::calculateStateSpaceTrapezoidalMatrices(mA, mB, mE, mTimeStep, mdA, mdB,
                                               mdE);
  mW = mC * mdB + mD;
}

const Matrix &EMT::VTypeVariableSSNComp::stateOffset() const { return mE; }

const Matrix &EMT::VTypeVariableSSNComp::outputOffset() const { return mF; }

void EMT::VTypeVariableSSNComp::setStateOffset(const Matrix &E) {
  if (E.rows() != mA.rows() || E.cols() != 1)
    throw std::invalid_argument("State offset vector has invalid dimensions.");

  mE = E;
}

void EMT::VTypeVariableSSNComp::setOutputOffset(const Matrix &F) {
  if (F.rows() != mF.rows() || F.cols() != 1)
    throw std::invalid_argument("Output offset vector has invalid dimensions.");

  mF = F;
}

void EMT::VTypeVariableSSNComp::setParameters(const Matrix &A, const Matrix &B,
                                              const Matrix &C,
                                              const Matrix &D) {
  setParameters(A, B, C, D, Matrix::Zero(A.rows(), 1),
                Matrix::Zero(C.rows(), 1));
}

void EMT::VTypeVariableSSNComp::setParameters(const Matrix &A, const Matrix &B,
                                              const Matrix &C, const Matrix &D,
                                              const Matrix &E) {
  setParameters(A, B, C, D, E, Matrix::Zero(C.rows(), 1));
}

void EMT::VTypeVariableSSNComp::setParameters(const Matrix &A, const Matrix &B,
                                              const Matrix &C, const Matrix &D,
                                              const Matrix &E,
                                              const Matrix &F) {
  SSNComp::setParameters(A, B, C, D);
  setStateOffset(E);
  setOutputOffset(F);
  mdE = Matrix::Zero(mE.rows(), 1);
}

void EMT::VTypeVariableSSNComp::updateStateSpaceModel() {
  mParameterChanged = updateComponentParameters();

  if (mParameterChanged)
    recomputeDiscreteModel();
}

Bool EMT::VTypeVariableSSNComp::hasParameterChanged() {
  return mParameterChanged;
}

void EMT::VTypeVariableSSNComp::initializeFromNodesAndTerminals(
    Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  const MatrixComp uInit = buildInitialInputFromNodes(frequency);

  **mIntfVoltage = uInit.real();
  **mX = Matrix::Zero((**mX).rows(), 1);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from nodes and terminals ---");

  // Fixed-point initialization for operating-point-dependent SSN models.
  // The initial input is reconstructed from terminal quantities, then the
  // component parameters are updated from the current state estimate and the
  // steady-state state is recomputed. This is repeated until the state no
  // longer changes noticeably or until the iteration limit is reached.
  Bool converged = false;

  for (Int iter = 0; iter < mInitializationMaxIterations; ++iter) {
    const Matrix xPrev = **mX;

    updateComponentParameters();

    const MatrixComp xInit =
        calculateSteadyStateStateFromInput(uInit, frequency);
    **mX = xInit.real();

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

  // Ensure the local model corresponds to the final initialized state.
  updateComponentParameters();

  const MatrixComp xInit = (**mX).cast<Complex>();
  const MatrixComp yInit = calculateSteadyStateOutputFromInput(xInit, uInit);

  **mIntfCurrent = yInit.real();
  updateLogAttributes(**mIntfVoltage);
  mParameterChanged = false;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nInput u: {:s}"
      "\nOutput y: {:s}"
      "\nState x: {:s}"
      "\n--- Initialization from nodes and terminals finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent), Logger::matrixToString(**mX));
}
