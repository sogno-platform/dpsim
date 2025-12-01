/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph3_PiLine.h>

using namespace CPS;

DP::Ph3::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : Base::Ph3::PiLine(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(1);
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

///DEPRECATED: Remove method
SimPowerComp<Complex>::Ptr DP::Ph3::PiLine::clone(String name) {
  auto copy = PiLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void DP::Ph3::PiLine::initializeFromNodesAndTerminals(Real frequency) {

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
  **mIntfCurrent = iInit;
  **mIntfVoltage = vInitABC;

  // Initialization of virtual node
  // Initial voltage of phase B,C is set after A
  MatrixComp vInitTerm0 = MatrixComp::Zero(3, 1);
  vInitTerm0(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitTerm0(1, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_B;
  vInitTerm0(2, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_C;

  mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH *
                                      (vInitTerm0 + **mSeriesRes * iInit));

  // Create series sub components
  mSubSeriesResistor =
      std::make_shared<DP::Ph3::Resistor>(**mName + "_res", mLogLevel);
  mSubSeriesResistor->setParameters(**mSeriesRes);
  mSubSeriesResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  mSubSeriesResistor->initialize(mFrequencies);
  mSubSeriesResistor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubSeriesResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubSeriesInductor =
      std::make_shared<DP::Ph3::Inductor>(**mName + "_ind", mLogLevel);
  mSubSeriesInductor->setParameters(**mSeriesInd);
  mSubSeriesInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  mSubSeriesInductor->initialize(mFrequencies);
  mSubSeriesInductor->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubSeriesInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // By default there is always a small conductance to ground to
  // avoid problems with floating nodes.
  Matrix defaultParallelCond = Matrix::Zero(3, 3);
  defaultParallelCond << 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6;
  **mParallelCond =
      ((**mParallelCond)(0, 0) > 0) ? **mParallelCond : defaultParallelCond;

  // Create parallel sub components
  mSubParallelResistor0 =
      std::make_shared<DP::Ph3::Resistor>(**mName + "_con0", mLogLevel);
  mSubParallelResistor0->setParameters(2. * (**mParallelCond).inverse());
  mSubParallelResistor0->connect(
      SimNode::List{SimNode::GND, mTerminals[0]->node()});
  mSubParallelResistor0->initialize(mFrequencies);
  mSubParallelResistor0->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubParallelResistor0,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubParallelResistor1 =
      std::make_shared<DP::Ph3::Resistor>(**mName + "_con1", mLogLevel);
  mSubParallelResistor1->setParameters(2. * (**mParallelCond).inverse());
  mSubParallelResistor1->connect(
      SimNode::List{SimNode::GND, mTerminals[1]->node()});
  mSubParallelResistor1->initialize(mFrequencies);
  mSubParallelResistor1->initializeFromNodesAndTerminals(frequency);
  addMNASubComponent(mSubParallelResistor1,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  if ((**mParallelCap)(0, 0) > 0) {
    mSubParallelCapacitor0 =
        std::make_shared<DP::Ph3::Capacitor>(**mName + "_cap0", mLogLevel);
    mSubParallelCapacitor0->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    mSubParallelCapacitor0->initialize(mFrequencies);
    mSubParallelCapacitor0->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelCapacitor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubParallelCapacitor1 =
        std::make_shared<DP::Ph3::Capacitor>(**mName + "_cap1", mLogLevel);
    mSubParallelCapacitor1->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor1->connect(
        SimNode::List{SimNode::GND, mTerminals[1]->node()});
    mSubParallelCapacitor1->initialize(mFrequencies);
    mSubParallelCapacitor1->initializeFromNodesAndTerminals(frequency);
    addMNASubComponent(mSubParallelCapacitor1,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nTerminal 1 voltage: {:s}"
      "\nVirtual Node 1 voltage: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::matrixCompToString(**mIntfVoltage),
      Logger::matrixCompToString(**mIntfCurrent),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
      Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)),
      Logger::phasorToString(mVirtualNodes[0]->initialSingleVoltage()));
}

void DP::Ph3::PiLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // add pre-step dependencies of component itself
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph3::PiLine::mnaParentPreStep(Real time, Int timeStepCount) {
  // pre-step of component itself
  this->mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::PiLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  // add post-step dependencies of component itself
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::PiLine::mnaParentPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  // post-step of component itself
  this->mnaUpdateVoltage(**leftVector);
  this->mnaUpdateCurrent(**leftVector);
}

void DP::Ph3::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}

void DP::Ph3::PiLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = mSubSeriesInductor->intfCurrent();
}

// #### Tear Methods ####
MNAInterface::List DP::Ph3::PiLine::mnaTearGroundComponents() {
  MNAInterface::List gndComponents;

  gndComponents.push_back(mSubParallelResistor0);
  gndComponents.push_back(mSubParallelResistor1);

  if ((**mParallelCap)(0, 0) > 0) {
    gndComponents.push_back(mSubParallelCapacitor0);
    gndComponents.push_back(mSubParallelCapacitor1);
  }

  return gndComponents;
}

void DP::Ph3::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
  mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
  mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
  mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
  mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
}

void DP::Ph3::PiLine::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
  mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
}

void DP::Ph3::PiLine::mnaTearApplyVoltageStamp(Matrix &voltageVector) {
  mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
}

void DP::Ph3::PiLine::mnaTearPostStep(MatrixComp voltage, MatrixComp current) {
  mSubSeriesInductor->mnaTearPostStep(voltage - (**mSeriesRes * current),
                                      current);
  (**mIntfCurrent) = mSubSeriesInductor->intfCurrent();
}
