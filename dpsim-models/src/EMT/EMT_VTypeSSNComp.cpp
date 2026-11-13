// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

using namespace CPS;

EMT::VTypeSSNComp::VTypeSSNComp(String uid, String name, Int inputSize,
                                Int outputSize, Logger::Level logLevel)
    : SSNComp(uid, name, inputSize, outputSize, logLevel) {
  *mIntfVoltageValue = Matrix::Zero(inputSize, 1);
  *mIntfCurrentValue = Matrix::Zero(outputSize, 1);

  // Some nonlinear SSN components assemble their initial history while
  // initializing from the power-flow operating point, before the MNA solver
  // calls mnaCompInitialize(). Bind the stable attribute storage here so
  // stateValue()/inputValue() are valid during that initialization path too.
  mXValue = &mX->get();
  mInputValue = &mIntfVoltage->get();
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

  mX->get() = xInit.real();
  *mIntfVoltageValue = uInit.real();
  *mIntfCurrentValue = yInit.real();
  updateLogAttributes(*mIntfVoltageValue);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from nodes and terminals ---"
      "\nInput u: {:s}"
      "\nOutput y: {:s}"
      "\nState x: {:s}"
      "\n--- Initialization from nodes and terminals finished ---",
      Logger::matrixToString(*mIntfVoltageValue),
      Logger::matrixToString(*mIntfCurrentValue),
      Logger::matrixToString(mX->get()));
}

void EMT::VTypeSSNComp::mnaCompUpdateCurrent(const Matrix &) {
  *mIntfCurrentValue = mW * (*mIntfVoltageValue) + mYHist;
}

void EMT::VTypeSSNComp::mnaCompPostStep(Real, Int,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mPreviousInput = *mIntfVoltageValue;
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  updateState(mPreviousInput, *mIntfVoltageValue);
  updateLogAttributes(*mIntfVoltageValue);
}
