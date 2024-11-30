/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 ******************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_ExponentialDiode.h>

using namespace CPS;

EMT::Ph1::ExponentialDiode::ExponentialDiode(String uid, String name,
                                             Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, false, false, logLevel),
      mI_S(mAttributes->create<Real>("I_S")),
      mV_T(mAttributes->create<Real>("V_T")) {
  mPhaseType = PhaseType::Single;
  setTerminalNumber(2);
  **mI_S = 0.000001;
  **mV_T = 0.027;
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

void EMT::Ph1::ExponentialDiode::setParameters(Real I_S, Real V_T) {
  **mI_S = I_S;
  **mV_T = V_T;
}

SimPowerComp<Real>::Ptr EMT::Ph1::ExponentialDiode::clone(String name) {
  auto copy = ExponentialDiode::make(name, mLogLevel);
  copy->setParameters(**mI_S, **mV_T);
  return copy;
}

void EMT::Ph1::ExponentialDiode::initializeFromNodesAndTerminals(
    Real frequency) {

  // IntfVoltage initialization for each phase
  MatrixComp vInitABC = Matrix::Zero(1, 1);
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  (**mIntfVoltage)(0, 0) = vInitABC(0, 0).real();

  ///FIXME: 	Initialization should include solving the system once to obtain
  //			the actual values solving the
  //			system for the 0th time step. As of now abnormal current
  //			values for the 0th time step are to be expected.

  (**mIntfCurrent)(0, 0) =
      (**mI_S) * (expf((**mIntfVoltage)(0, 0) / (**mV_T)) - 1.);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:f}"
                     "\nCurrent: {:f}"
                     "\nTerminal 0 voltage: {:f}"
                     "\nTerminal 1 voltage: {:f}"
                     "\n--- Initialization from powerflow finished ---",
                     (**mIntfVoltage)(0, 0), (**mIntfCurrent)(0, 0),
                     initialSingleVoltage(0).real(),
                     initialSingleVoltage(1).real());
}

void EMT::Ph1::ExponentialDiode::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

  updateMatrixNodeIndices();

  **mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
  **mNonlinearFunctionStamp = Matrix::Zero(leftVector->get().rows(), 1);
  calculateNonlinearFunctionResult();
  updateJacobian();

  mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph1::ExponentialDiode::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  // Set diagonal entries
  if (terminalNotGrounded(0)) {
    // set upper left block, 3x3 entries
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(0, 0), Jacobian(0, 0));
  }
  if (terminalNotGrounded(1)) {
    // set buttom right block, 3x3 entries
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(1, 0), Jacobian(0, 0));
  }
  // Set off diagonal blocks, 2x3x3 entries
  if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0),
                             matrixNodeIndex(1, 0), -Jacobian(0, 0));

    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0),
                             matrixNodeIndex(0, 0), -Jacobian(0, 0));
  }
}

void EMT::Ph1::ExponentialDiode::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(attribute("v_intf"));
  modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph1::ExponentialDiode::mnaCompPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::ExponentialDiode::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaUpdateVoltage(**leftVector);
  mnaUpdateCurrent(**leftVector);
}

void EMT::Ph1::ExponentialDiode::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  }
}

void EMT::Ph1::ExponentialDiode::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) =
      (**mI_S) * (expf((-1.0) * (**mIntfVoltage)(0, 0) / (**mV_T)) - 1.);
}

void EMT::Ph1::ExponentialDiode::iterationUpdate(const Matrix &leftVector) {
  //Update phase voltages
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  }

  calculateNonlinearFunctionResult();

  updateJacobian();
}

void EMT::Ph1::ExponentialDiode::calculateNonlinearFunctionResult() {

  (**mIntfCurrent)(0, 0) =
      (**mI_S) * (expf((-1.0) * (**mIntfVoltage)(0, 0) / (**mV_T)) - 1.);

  if (terminalNotGrounded(1)) {
    Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(1, 0),
                           (**mIntfCurrent)(0, 0));
  }
  if (terminalNotGrounded(0)) {
    Math::setVectorElement(**mNonlinearFunctionStamp, matrixNodeIndex(0, 0),
                           -(**mIntfCurrent)(0, 0));
  }
}

void EMT::Ph1::ExponentialDiode::updateJacobian() {
  Jacobian(0, 0) =
      (**mI_S / (**mV_T)) *(-1.0) * expf((-1.0) * (**mIntfVoltage)(0, 0) / (**mV_T));
}