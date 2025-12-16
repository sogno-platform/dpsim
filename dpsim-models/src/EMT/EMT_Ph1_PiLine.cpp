/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph1_PiLine.h>

using namespace CPS;

EMT::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : Base::Ph1::PiLine(mAttributes),
      CompositePowerComp<Real>(uid, name, true, true, logLevel) {
  setVirtualNodeNumber(1);
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);

  mSLog->flush();
}

/// DEPRECATED: Delete method
SimPowerComp<Real>::Ptr EMT::Ph1::PiLine::clone(String name) {
  auto copy = PiLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void EMT::Ph1::PiLine::initializeFromNodesAndTerminals(Real frequency) {

  // By default there is always a small conductance to ground to
  // avoid problems with floating nodes.

  Real defaultParallelCond = 1e-6;
  **mParallelCond =
      (**mParallelCond > 0) ? **mParallelCond : defaultParallelCond;

  // Static calculation
  Real omega = 2. * PI * frequency;
  Complex impedance = {**mSeriesRes, omega * **mSeriesInd};
  Complex voltage =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  (**mIntfVoltage)(0, 0) = voltage.real();
  (**mIntfCurrent)(0, 0) = (voltage / impedance).real();

  // Initialization of virtual node
  mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) +
                                      (**mIntfCurrent)(0, 0) * **mSeriesRes);

  // Create series sub components
  mSubSeriesResistor =
      std::make_shared<EMT::Ph1::Resistor>(**mName + "_res", mLogLevel);
  mSubSeriesResistor->setParameters(**mSeriesRes);
  mSubSeriesResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  mSubSeriesResistor->initialize(mFrequencies);
  mSubSeriesResistor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubSeriesResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubSeriesInductor =
      std::make_shared<EMT::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubSeriesInductor->setParameters(**mSeriesInd);
  mSubSeriesInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  mSubSeriesInductor->initialize(mFrequencies);
  mSubSeriesInductor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubSeriesInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // Create parallel sub components
  mSubParallelResistor0 =
      std::make_shared<EMT::Ph1::Resistor>(**mName + "_con0", mLogLevel);
  mSubParallelResistor0->setParameters(2. / (**mParallelCond));
  mSubParallelResistor0->connect(
      SimNode::List{SimNode::GND, mTerminals[0]->node()});
  mSubParallelResistor0->initialize(mFrequencies);
  mSubParallelResistor0->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubParallelResistor0,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubParallelResistor1 =
      std::make_shared<EMT::Ph1::Resistor>(**mName + "_con1", mLogLevel);
  mSubParallelResistor1->setParameters(2. / (**mParallelCond));
  mSubParallelResistor1->connect(
      SimNode::List{SimNode::GND, mTerminals[1]->node()});
  mSubParallelResistor1->initialize(mFrequencies);
  mSubParallelResistor1->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubParallelResistor1,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  if ((**mParallelCap) > 0) {
    mSubParallelCapacitor0 =
        std::make_shared<EMT::Ph1::Capacitor>(**mName + "_cap0", mLogLevel);
    mSubParallelCapacitor0->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    mSubParallelCapacitor0->initialize(mFrequencies);
    mSubParallelCapacitor0->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelCapacitor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubParallelCapacitor1 =
        std::make_shared<EMT::Ph1::Capacitor>(**mName + "_cap1", mLogLevel);
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
                      "\n Impedance: {:s}",
                      Logger::matrixToString(**mSeriesRes),
                      Logger::matrixToString(**mSeriesInd),
                      Logger::complexToString(impedance));

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nTerminal 1 voltage: {:s}"
      "\nVirtual Node 1 voltage: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::matrixToString(**mIntfVoltage),
      Logger::matrixToString(**mIntfCurrent),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)),
      Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
  mSLog->flush();
}

void EMT::Ph1::PiLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph1::PiLine::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::PiLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::PiLine::mnaParentPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::PiLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mSubSeriesInductor->intfCurrent();
}

// #### Tear Methods ####
MNAInterface::List EMT::Ph1::PiLine::mnaTearGroundComponents() {
  MNAInterface::List gndComponents;

  gndComponents.push_back(mSubParallelResistor0);
  gndComponents.push_back(mSubParallelResistor1);

  if ((**mParallelCap) > 0) {
    gndComponents.push_back(mSubParallelCapacitor0);
    gndComponents.push_back(mSubParallelCapacitor1);
  }

  return gndComponents;
}

void EMT::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
  mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
  mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
  mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
  mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
}

void EMT::Ph1::PiLine::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
  mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
}

void EMT::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix &voltageVector) {
  mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
}

void EMT::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
  mSubSeriesInductor->mnaTearPostStep(voltage - current * **mSeriesRes,
                                      current);
  (**mIntfCurrent) = mSubSeriesInductor->intfCurrent();
}
