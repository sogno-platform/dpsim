// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_VTypeSSNComp.h>

using namespace CPS;

DP::VTypeSSNComp::VTypeSSNComp(String uid, String name, Int inputSize,
                               Int outputSize, Logger::Level logLevel)
    : SSNComp(uid, name, inputSize, outputSize, logLevel) {
  **mIntfVoltage = MatrixComp::Zero(inputSize, 1);
  **mIntfCurrent = MatrixComp::Zero(outputSize, 1);
}

Attribute<MatrixComp>::Ptr DP::VTypeSSNComp::inputAttribute() const {
  return mIntfVoltage;
}

Attribute<MatrixComp>::Ptr DP::VTypeSSNComp::outputAttribute() const {
  return mIntfCurrent;
}

void DP::VTypeSSNComp::initializeFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  Real omega = 2. * PI * frequency;
  MatrixComp uInit = buildInitialInputFromNodes(frequency);
  MatrixComp xInit = calculateSteadyStateStateFromInput(uInit, omega);
  MatrixComp yInit = calculateSteadyStateOutputFromInput(xInit, uInit);

  **mX = xInit;
  **mIntfVoltage = uInit;
  **mIntfCurrent = yInit;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from nodes and terminals ---"
                     "\nInput u: {:s}"
                     "\nOutput y: {:s}"
                     "\nState x: {:s}"
                     "\n--- Initialization finished ---",
                     Logger::matrixCompToString(**mIntfVoltage),
                     Logger::matrixCompToString(**mIntfCurrent),
                     Logger::matrixCompToString(**mX));
}

void DP::VTypeSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  **mIntfCurrent = mW * (**mIntfVoltage) + mYHist;
}

void DP::VTypeSSNComp::mnaCompPostStep(Real, Int,
                                       Attribute<Matrix>::Ptr &leftVector) {
  MatrixComp uOld = **mIntfVoltage;
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  updateState(uOld, **mIntfVoltage);
}
