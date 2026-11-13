/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph1_PiLine.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

EMT::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : PiLine(uid, name, true, logLevel) {}

EMT::Ph1::PiLine::PiLine(String uid, String name, Bool useSubcomponents,
                         Logger::Level logLevel)
    : Base::Ph1::PiLine(mAttributes),
      CompositePowerComp<Real>(uid, name, true, true, logLevel) {
  mUseSubcomponents = useSubcomponents;
  // Keep the legacy virtual-node allocation so the matrix graph has the same
  // dimensions in either mode; the direct realization leaves that variable
  // isolated with a unit diagonal.
  setVirtualNodeNumber(1);
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);

  mSLog->flush();
}

/// DEPRECATED: Delete method
SimPowerComp<Real>::Ptr EMT::Ph1::PiLine::clone(String name) {
  auto copy = PiLine::make(name, mUseSubcomponents, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void EMT::Ph1::PiLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  // By default there is always a small conductance to ground to
  // avoid problems with floating nodes.
  Real defaultParallelCond = 1e-6;
  **mParallelCond =
      (**mParallelCond > 0) ? **mParallelCond : defaultParallelCond;

  if (!mUseSubcomponents)
    return;

  // Create series sub components
  mSubSeriesResistor =
      std::make_shared<EMT::Ph1::Resistor>(**mName + "_res", mLogLevel);
  mSubSeriesResistor->setParameters(**mSeriesRes);
  mSubSeriesResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  addMNASubComponent(mSubSeriesResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubSeriesInductor =
      std::make_shared<EMT::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubSeriesInductor->setParameters(**mSeriesInd);
  mSubSeriesInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  addMNASubComponent(mSubSeriesInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  // Create parallel sub components
  mSubParallelResistor0 =
      std::make_shared<EMT::Ph1::Resistor>(**mName + "_con0", mLogLevel);
  mSubParallelResistor0->setParameters(2. / (**mParallelCond));
  mSubParallelResistor0->connect(
      SimNode::List{SimNode::GND, mTerminals[0]->node()});
  addMNASubComponent(mSubParallelResistor0,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubParallelResistor1 =
      std::make_shared<EMT::Ph1::Resistor>(**mName + "_con1", mLogLevel);
  mSubParallelResistor1->setParameters(2. / (**mParallelCond));
  mSubParallelResistor1->connect(
      SimNode::List{SimNode::GND, mTerminals[1]->node()});
  addMNASubComponent(mSubParallelResistor1,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  if ((**mParallelCap) > 0) {
    mSubParallelCapacitor0 =
        std::make_shared<EMT::Ph1::Capacitor>(**mName + "_cap0", mLogLevel);
    mSubParallelCapacitor0->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    addMNASubComponent(mSubParallelCapacitor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubParallelCapacitor1 =
        std::make_shared<EMT::Ph1::Capacitor>(**mName + "_cap1", mLogLevel);
    mSubParallelCapacitor1->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor1->connect(
        SimNode::List{SimNode::GND, mTerminals[1]->node()});
    addMNASubComponent(mSubParallelCapacitor1,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }
}

void EMT::Ph1::PiLine::initializeParentFromNodesAndTerminals(Real frequency) {
  // Static calculation
  Real omega = 2. * PI * frequency;
  Complex impedance = {**mSeriesRes, omega * **mSeriesInd};
  Complex voltage =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  Complex current = voltage / impedance;
  (**mIntfVoltage)(0, 0) = voltage.real();
  (**mIntfCurrent)(0, 0) = current.real();

  // Initialization of virtual node for the legacy series R-L realization.
  if (mUseSubcomponents)
    mVirtualNodes[0]->setInitialVoltage(
        initialSingleVoltage(0) + PEAK1PH_TO_RMS3PH * current * **mSeriesRes);

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

void EMT::Ph1::PiLine::mnaParentInitialize(Real omega, Real timeStep,
                                           Attribute<Matrix>::Ptr leftVector) {
  if (mUseSubcomponents)
    return;

  updateMatrixNodeIndices();
  mDirectInductorConductance = timeStep / (2. * **mSeriesInd);
  mDirectSeriesConductance = mDirectInductorConductance /
                             (1. + mDirectInductorConductance * **mSeriesRes);
  mDirectShuntConductance = **mParallelCond / 2.;
  mDirectCapacitorConductance = **mParallelCap / timeStep;

  const Real initialSeriesCurrent = (**mIntfCurrent)(0, 0);
  const Real initialInductorVoltage =
      (**mIntfVoltage)(0, 0) - **mSeriesRes * initialSeriesCurrent;
  mDirectInductorHistory = mDirectInductorConductance * initialInductorVoltage +
                           initialSeriesCurrent;
  mDirectSeriesHistory =
      mDirectInductorHistory / (1. + mDirectInductorConductance * **mSeriesRes);

  if (**mParallelCap > 0.) {
    const Real capacitance = **mParallelCap / 2.;
    const Complex capacitorImpedance(0., -1. / (omega * capacitance));
    const Complex initial0 = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
    const Complex initial1 = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1);
    const Real current0 = (initial0 / capacitorImpedance).real();
    const Real current1 = (initial1 / capacitorImpedance).real();
    mDirectCapacitorHistory0 =
        current0 + mDirectCapacitorConductance * initial0.real();
    mDirectCapacitorHistory1 =
        current1 + mDirectCapacitorConductance * initial1.real();
  }
}

void EMT::Ph1::PiLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  if (!mUseSubcomponents) {
    modifiedAttributes.push_back(mRightVector);
    return;
  }
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph1::PiLine::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::PiLine::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;

  MNAStampUtils::stampConductance(mDirectSeriesConductance, systemMatrix,
                                  matrixNodeIndex(0), matrixNodeIndex(1),
                                  terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
  Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                           mVirtualNodes[0]->matrixNodeIndex(), 1.);
  const Real shuntAdmittance =
      mDirectShuntConductance + mDirectCapacitorConductance;
  if (terminalNotGrounded(0))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             matrixNodeIndex(0), shuntAdmittance);
  if (terminalNotGrounded(1))
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             matrixNodeIndex(1), shuntAdmittance);
}

void EMT::Ph1::PiLine::mnaParentApplyRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents)
    return;

  if (terminalNotGrounded(0)) {
    Math::addToVectorElement(rightVector, matrixNodeIndex(0),
                             mDirectSeriesHistory);
    Math::addToVectorElement(rightVector, matrixNodeIndex(0),
                             mDirectCapacitorHistory0);
  }
  if (terminalNotGrounded(1)) {
    Math::addToVectorElement(rightVector, matrixNodeIndex(1),
                             -mDirectSeriesHistory);
    Math::addToVectorElement(rightVector, matrixNodeIndex(1),
                             mDirectCapacitorHistory1);
  }
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

  if (!mUseSubcomponents) {
    const Real seriesCurrent = (**mIntfCurrent)(0, 0);
    const Real inductorVoltage =
        (**mIntfVoltage)(0, 0) - **mSeriesRes * seriesCurrent;
    mDirectInductorHistory =
        mDirectInductorConductance * inductorVoltage + seriesCurrent;
    mDirectSeriesHistory = mDirectInductorHistory /
                           (1. + mDirectInductorConductance * **mSeriesRes);
    const Real terminal0Voltage =
        terminalNotGrounded(0)
            ? Math::realFromVectorElement(**leftVector, matrixNodeIndex(0))
            : 0.;
    const Real terminal1Voltage =
        terminalNotGrounded(1)
            ? Math::realFromVectorElement(**leftVector, matrixNodeIndex(1))
            : 0.;
    mDirectCapacitorHistory0 =
        2. * mDirectCapacitorConductance * terminal0Voltage -
        mDirectCapacitorHistory0;
    mDirectCapacitorHistory1 =
        2. * mDirectCapacitorConductance * terminal1Voltage -
        mDirectCapacitorHistory1;
  }
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
  if (mUseSubcomponents)
    **mIntfCurrent = mSubSeriesInductor->intfCurrent();
  else
    (**mIntfCurrent)(0, 0) = mDirectSeriesConductance * (**mIntfVoltage)(0, 0) +
                             mDirectSeriesHistory;
}

// #### Tear Methods ####
MNAInterface::List EMT::Ph1::PiLine::mnaTearGroundComponents() {
  MNAInterface::List gndComponents;

  if (!mUseSubcomponents)
    return gndComponents;

  gndComponents.push_back(mSubParallelResistor0);
  gndComponents.push_back(mSubParallelResistor1);

  if ((**mParallelCap) > 0) {
    gndComponents.push_back(mSubParallelCapacitor0);
    gndComponents.push_back(mSubParallelCapacitor1);
  }

  return gndComponents;
}

void EMT::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
  if (!mUseSubcomponents) {
    mDirectInductorConductance = timeStep / (2. * **mSeriesInd);
    mDirectSeriesConductance = mDirectInductorConductance /
                               (1. + mDirectInductorConductance * **mSeriesRes);
    return;
  }
  mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
  mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
  mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
  mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
}

void EMT::Ph1::PiLine::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  if (!mUseSubcomponents) {
    Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx,
                             **mSeriesRes + 1. / mDirectInductorConductance);
    return;
  }
  mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
  mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
}

void EMT::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix &voltageVector) {
  if (!mUseSubcomponents) {
    Math::addToVectorElement(voltageVector, mTearIdx,
                             mDirectInductorHistory /
                                 mDirectInductorConductance);
    return;
  }
  mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
}

void EMT::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
  if (!mUseSubcomponents) {
    const Real realCurrent = current.real();
    const Real inductorVoltage = voltage.real() - realCurrent * **mSeriesRes;
    mDirectInductorHistory =
        mDirectInductorConductance * inductorVoltage + realCurrent;
    (**mIntfVoltage)(0, 0) = voltage.real();
    (**mIntfCurrent)(0, 0) = realCurrent;
    return;
  }
  mSubSeriesInductor->mnaTearPostStep(voltage - current * **mSeriesRes,
                                      current);
  (**mIntfCurrent) = mSubSeriesInductor->intfCurrent();
}
