// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_VTypeVariableSSNComp.h>

using namespace CPS;

EMT::VTypeVariableSSNComp::VTypeVariableSSNComp(String uid, String name,
                                                Int inputSize, Int outputSize,
                                                Logger::Level logLevel)
    : VTypeSSNComp(uid, name, inputSize, outputSize, logLevel),
      mParameterChanged(false), mF(Matrix::Zero(outputSize, 1)) {}

Matrix EMT::VTypeVariableSSNComp::calculateHistoryVector() const {
  return mC * (mdA * (**mX) + mdB * (**inputAttribute())) + mF;
}

const Matrix &EMT::VTypeVariableSSNComp::outputOffset() const { return mF; }

void EMT::VTypeVariableSSNComp::setOutputOffset(const Matrix &F) {
  if (F.rows() != mF.rows() || F.cols() != 1)
    throw std::invalid_argument("Output offset vector has invalid dimensions.");

  mF = F;
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
  // This keeps the implementation generic while still allowing the component
  // parameters to depend on the current state.
  constexpr Int maxIterations = 10;
  constexpr Real tolerance = 1e-9;
  Bool converged = false;

  for (Int iter = 0; iter < maxIterations; ++iter) {
    const Matrix xPrev = **mX;

    updateComponentParameters();

    const MatrixComp xInit =
        calculateSteadyStateStateFromInput(uInit, frequency);
    **mX = xInit.real();

    if ((**mX).isApprox(xPrev, tolerance)) {
      converged = true;
      break;
    }
  }

  if (!converged) {
    SPDLOG_LOGGER_WARN(
        mSLog,
        "Fixed-point initialization did not converge within {} iterations.",
        maxIterations);
  }

  // Ensure the output matrices correspond to the final initialized state.
  updateComponentParameters();

  const MatrixComp xInit = (**mX).cast<Complex>();
  const MatrixComp yInit = mC.cast<Complex>() * xInit +
                           mD.cast<Complex>() * uInit + mF.cast<Complex>();

  **mIntfCurrent = yInit.real();
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
