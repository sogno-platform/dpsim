/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_RxLine.h>

using namespace CPS;

DP::Ph1::RxLine::RxLine(String uid, String name, Logger::Level logLevel)
    : Base::Ph1::PiLine(mAttributes), CompositePowerComp<Complex>(
                                          uid, name, true, true, logLevel) {
  setVirtualNodeNumber(1);
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

///DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr DP::Ph1::RxLine::clone(String name) {
  auto copy = RxLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd);
  return copy;
}

void DP::Ph1::RxLine::initializeFromNodesAndTerminals(Real frequency) {

  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  Complex impedance = {**mSeriesRes, **mSeriesInd * 2. * PI * frequency};
  (**mIntfCurrent)(0, 0) = 0;
  mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) +
                                      (**mIntfCurrent)(0, 0) * **mSeriesRes);

  // Default model with virtual node in between
  mSubResistor =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
  mSubResistor->setParameters(**mSeriesRes);
  mSubResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  mSubResistor->initialize(mFrequencies);
  mSubResistor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubInductor =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(**mSeriesInd);
  mSubInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  mSubInductor->initialize(mFrequencies);
  mSubInductor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mInitialResistor =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_snubber_res", mLogLevel);
  mInitialResistor->setParameters(1e6);
  mInitialResistor->connect({SimNode::GND, mTerminals[1]->node()});
  mInitialResistor->initialize(mFrequencies);
  mInitialResistor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mInitialResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

void DP::Ph1::RxLine::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::RxLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::RxLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph1::RxLine::mnaParentPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph1::RxLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::RxLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mSubInductor->intfCurrent()(0, 0);
}
