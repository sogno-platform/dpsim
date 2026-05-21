// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

using namespace CPS;

EMT::VTypeSSNComp::VTypeSSNComp(String uid, String name, Int inputSize,
                                Int outputSize, Logger::Level logLevel)
    : SSNComp(uid, name, inputSize, outputSize, logLevel) {
  **mIntfVoltage = Matrix::Zero(inputSize, 1);
  **mIntfCurrent = Matrix::Zero(outputSize, 1);
}

Attribute<Matrix>::Ptr EMT::VTypeSSNComp::inputAttribute() const {
  return mIntfVoltage;
}

Attribute<Matrix>::Ptr EMT::VTypeSSNComp::outputAttribute() const {
  return mIntfCurrent;
}

void EMT::VTypeSSNComp::initializeFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  MatrixComp uInit = buildInitialInputFromNodes(frequency);
  MatrixComp xInit = calculateSteadyStateStateFromInput(uInit, frequency);
  MatrixComp yInit = calculateSteadyStateOutputFromInput(xInit, uInit);

  **mX = xInit.real();
  **mIntfVoltage = uInit.real();
  **mIntfCurrent = yInit.real();
  updateLogAttributes(**mIntfVoltage);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from nodes and terminals ---"
      "\nInput u: {:s}"
      "\nOutput y: {:s}"
      "\nState x: {:s}"
      "\n--- Initialization from nodes and terminals finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent), Logger::matrixToString(**mX));
}

void EMT::VTypeSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  **mIntfCurrent = mW * (**mIntfVoltage) + mYHist;
}

void EMT::VTypeSSNComp::mnaCompPostStep(Real, Int,
                                        Attribute<Matrix>::Ptr &leftVector) {
  Matrix uOld = **mIntfVoltage;
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  updateState(uOld, **mIntfVoltage);
  updateLogAttributes(**mIntfVoltage);
}
