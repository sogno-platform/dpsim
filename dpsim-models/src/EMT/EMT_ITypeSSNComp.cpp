// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_ITypeSSNComp.h>

using namespace CPS;

EMT::ITypeSSNComp::ITypeSSNComp(String uid, String name, Int inputSize,
                                Int outputSize, Logger::Level logLevel)
    : SSNComp(uid, name, inputSize, outputSize, logLevel) {
  **mIntfCurrent = Matrix::Zero(inputSize, 1);
  **mIntfVoltage = Matrix::Zero(outputSize, 1);
}

Attribute<Matrix>::Ptr EMT::ITypeSSNComp::inputAttribute() const {
  return mIntfCurrent;
}

Attribute<Matrix>::Ptr EMT::ITypeSSNComp::outputAttribute() const {
  return mIntfVoltage;
}

void EMT::ITypeSSNComp::initializeFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  MatrixComp uInit = buildInitialInputFromNodes(frequency);
  MatrixComp xInit = calculateSteadyStateStateFromInput(uInit, frequency);
  MatrixComp yInit = calculateSteadyStateOutputFromInput(xInit, uInit);

  **mX = xInit.real();
  **mIntfCurrent = uInit.real();
  **mIntfVoltage = yInit.real();

  updateLogAttributes(**mIntfCurrent);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from nodes and terminals ---"
      "\nInput u: {:s}"
      "\nOutput y: {:s}"
      "\nState x: {:s}"
      "\n--- Initialization from nodes and terminals finished ---",
      Logger::matrixToString(**mIntfCurrent),
      Logger::matrixToString(**mIntfVoltage), Logger::matrixToString(**mX));
}

void EMT::ITypeSSNComp::mnaCompUpdateVoltage(const Matrix &) {}

void EMT::ITypeSSNComp::mnaCompPostStep(Real, Int,
                                        Attribute<Matrix>::Ptr &leftVector) {
  Matrix uOld = **mIntfCurrent;

  // First reconstruct output voltage from the MNA solution.
  mnaCompUpdateVoltage(**leftVector);

  // Then reconstruct input current from:
  //   v = mW * i + mYHist
  mnaCompUpdateCurrent(**leftVector);

  updateState(uOld, **mIntfCurrent);
  updateLogAttributes(**mIntfCurrent);
}
