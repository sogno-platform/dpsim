/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_PiLine.h>

using namespace CPS;

EMT::Ph3::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : Base::Ph3::PiLine(mAttributes),
      CompositePowerComp<Real>(uid, name, true, true, logLevel) {
  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  mSLog->flush();
}

/// DEPRECATED: Delete method
SimPowerComp<Real>::Ptr EMT::Ph3::PiLine::clone(String name) {
  auto copy = PiLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void EMT::Ph3::PiLine::initializeFromNodesAndTerminals(Real frequency) {

  // Static calculation
  Real omega = 2. * PI * frequency;
  MatrixComp impedance = MatrixComp::Zero(3, 3);
  impedance << Complex((**mSeriesRes)(0, 0), omega * (**mSeriesInd)(0, 0)),
      Complex((**mSeriesRes)(0, 1), omega * (**mSeriesInd)(0, 1)),
      Complex((**mSeriesRes)(0, 2), omega * (**mSeriesInd)(0, 2)),
      Complex((**mSeriesRes)(1, 0), omega * (**mSeriesInd)(1, 0)),
      Complex((**mSeriesRes)(1, 1), omega * (**mSeriesInd)(1, 1)),
      Complex((**mSeriesRes)(1, 2), omega * (**mSeriesInd)(1, 2)),
      Complex((**mSeriesRes)(2, 0), omega * (**mSeriesInd)(2, 0)),
      Complex((**mSeriesRes)(2, 1), omega * (**mSeriesInd)(2, 1)),
      Complex((**mSeriesRes)(2, 2), omega * (**mSeriesInd)(2, 2));

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) -
                   RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
  MatrixComp iInit = impedance.inverse() * vInitABC;
  **mIntfCurrent = iInit.real();
  **mIntfVoltage = vInitABC.real();

  // Create series rl sub component
  mSubSeriesElement = std::make_shared<EMT::Ph3::ResIndSeries>(
      **mName + "_ResIndSeries", mLogLevel);
  mSubSeriesElement->connect({mTerminals[0]->node(), mTerminals[1]->node()});
  mSubSeriesElement->setParameters(**mSeriesRes, **mSeriesInd);
  mSubSeriesElement->initialize(mFrequencies);
  mSubSeriesElement->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubSeriesElement,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // Create parallel sub components
  if ((**mParallelCond)(0, 0) > 0) {
    mSubParallelResistor0 =
        std::make_shared<EMT::Ph3::Resistor>(**mName + "_con0", mLogLevel);
    mSubParallelResistor0->setParameters(2. * (**mParallelCond).inverse());
    mSubParallelResistor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    mSubParallelResistor0->initialize(mFrequencies);
    mSubParallelResistor0->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelResistor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

    mSubParallelResistor1 =
        std::make_shared<EMT::Ph3::Resistor>(**mName + "_con1", mLogLevel);
    mSubParallelResistor1->setParameters(2. * (**mParallelCond).inverse());
    mSubParallelResistor1->connect(
        SimNode::List{SimNode::GND, mTerminals[1]->node()});
    mSubParallelResistor1->initialize(mFrequencies);
    mSubParallelResistor1->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelResistor1,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  }

  if ((**mParallelCap)(0, 0) > 0) {
    mSubParallelCapacitor0 =
        std::make_shared<EMT::Ph3::Capacitor>(**mName + "_cap0", mLogLevel);
    mSubParallelCapacitor0->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    mSubParallelCapacitor0->initialize(mFrequencies);
    mSubParallelCapacitor0->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelCapacitor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubParallelCapacitor1 =
        std::make_shared<EMT::Ph3::Capacitor>(**mName + "_cap1", mLogLevel);
    mSubParallelCapacitor1->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor1->connect(
        SimNode::List{SimNode::GND, mTerminals[1]->node()});
    mSubParallelCapacitor1->initialize(mFrequencies);
    mSubParallelCapacitor1->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelCapacitor1,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

  SPDLOG_LOGGER_DEBUG(mSLog,
                      "\n--debug--"
                      "\n seriesRes: {:s}"
                      "\n seriesInd: {:s}"
                      "\n Impedance: {:s}"
                      "\n vInit: {:s}"
                      "\n iInit: {:s}",
                      Logger::matrixToString(**mSeriesRes),
                      Logger::matrixToString(**mSeriesInd),
                      Logger::matrixCompToString(impedance),
                      Logger::matrixCompToString(vInitABC),
                      Logger::matrixCompToString(iInit));

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nTerminal 1 voltage: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)));
  mSLog->flush();
}

void EMT::Ph3::PiLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::PiLine::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::PiLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::PiLine::mnaParentPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}

void EMT::Ph3::PiLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mSubSeriesElement->intfCurrent();
}