// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_ITypeSSNComp.h>

using namespace CPS;

DP::ITypeSSNComp::ITypeSSNComp(String uid, String name, Int inputSize,
                               Int outputSize, Logger::Level logLevel)
    : SSNComp(uid, name, inputSize, outputSize, logLevel) {
  **mIntfCurrent = MatrixComp::Zero(inputSize, 1);
  **mIntfVoltage = MatrixComp::Zero(outputSize, 1);
}

Attribute<MatrixComp>::Ptr DP::ITypeSSNComp::inputAttribute() const {
  return mIntfCurrent;
}

Attribute<MatrixComp>::Ptr DP::ITypeSSNComp::outputAttribute() const {
  return mIntfVoltage;
}

void DP::ITypeSSNComp::initializeFromNodesAndTerminals(Real frequency) {
  if (!mParametersSet)
    throw std::logic_error("setParameters() must be called before "
                           "initializeFromNodesAndTerminals().");

  Real omega = 2. * PI * frequency;
  MatrixComp uInit = buildInitialInputFromNodes(frequency);
  MatrixComp xInit = calculateSteadyStateStateFromInput(uInit, omega);
  MatrixComp yInit = calculateSteadyStateOutputFromInput(xInit, uInit);

  **mX = xInit;
  **mIntfCurrent = uInit;
  **mIntfVoltage = yInit;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from nodes and terminals ---"
                     "\nInput u: {:s}"
                     "\nOutput y: {:s}"
                     "\nState x: {:s}"
                     "\n--- Initialization finished ---",
                     Logger::matrixCompToString(**mIntfCurrent),
                     Logger::matrixCompToString(**mIntfVoltage),
                     Logger::matrixCompToString(**mX));
}

void DP::ITypeSSNComp::mnaCompUpdateVoltage(const Matrix &) {}

void DP::ITypeSSNComp::mnaCompPostStep(Real, Int,
                                       Attribute<Matrix>::Ptr &leftVector) {
  MatrixComp uOld = **mIntfCurrent;
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  updateState(uOld, **mIntfCurrent);
}
