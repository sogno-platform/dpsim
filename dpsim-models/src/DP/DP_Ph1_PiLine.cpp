/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph1_PiLine.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::PiLine::PiLine(String uid, String name, Logger::Level logLevel)
    : PiLine(uid, name, true, logLevel) {}

DP::Ph1::PiLine::PiLine(String uid, String name, Bool useSubcomponents,
                        Logger::Level logLevel)
    : Base::Ph1::PiLine(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mUseSubcomponents(useSubcomponents) {
  setVirtualNodeNumber(1);
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

///DEPRECATED: Remove method
SimPowerComp<Complex>::Ptr DP::Ph1::PiLine::clone(String name) {
  auto copy = PiLine::make(name, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd, **mParallelCap,
                      **mParallelCond);
  return copy;
}

void DP::Ph1::PiLine::createSubComponents() {
  if (!mUseSubcomponents)
    return;
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  // By default there is always a small conductance to ground to
  // avoid problems with floating nodes.
  Real defaultParallelCond = 1e-6;
  **mParallelCond =
      (**mParallelCond > 0) ? **mParallelCond : defaultParallelCond;

  mSubSeriesResistor =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
  mSubSeriesResistor->setParameters(**mSeriesRes);
  mSubSeriesResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  addMNASubComponent(mSubSeriesResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubSeriesInductor =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubSeriesInductor->setParameters(**mSeriesInd);
  mSubSeriesInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  addMNASubComponent(mSubSeriesInductor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubParallelResistor0 =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_con0", mLogLevel);
  mSubParallelResistor0->setParameters(2. / **mParallelCond);
  mSubParallelResistor0->connect(
      SimNode::List{SimNode::GND, mTerminals[0]->node()});
  addMNASubComponent(mSubParallelResistor0,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubParallelResistor1 =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_con1", mLogLevel);
  mSubParallelResistor1->setParameters(2. / **mParallelCond);
  mSubParallelResistor1->connect(
      SimNode::List{SimNode::GND, mTerminals[1]->node()});
  addMNASubComponent(mSubParallelResistor1,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  if (**mParallelCap > 0) {
    mSubParallelCapacitor0 =
        std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap0", mLogLevel);
    mSubParallelCapacitor0->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor0->connect(
        SimNode::List{SimNode::GND, mTerminals[0]->node()});
    addMNASubComponent(mSubParallelCapacitor0,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

    mSubParallelCapacitor1 =
        std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap1", mLogLevel);
    mSubParallelCapacitor1->setParameters(**mParallelCap / 2.);
    mSubParallelCapacitor1->connect(
        SimNode::List{SimNode::GND, mTerminals[1]->node()});
    addMNASubComponent(mSubParallelCapacitor1,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }
}

void DP::Ph1::PiLine::initializeParentFromNodesAndTerminals(Real frequency) {
  if (**mParallelCond <= 0.)
    **mParallelCond = 1e-6;
  mDirectSeriesConductance = (**mSeriesRes != 0.) ? 1. / **mSeriesRes : 0.;
  mDirectShuntConductance = **mParallelCond / 2.;
  mDirectShuntCapacitance = std::max(0., **mParallelCap) / 2.;

  // Static calculation
  Real omega = 2. * PI * frequency;
  Complex impedance = {**mSeriesRes, omega * **mSeriesInd};
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;

  // Initialization of virtual node
  if (mUseSubcomponents)
    mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) +
                                        (**mIntfCurrent)(0, 0) * **mSeriesRes);

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\n--- Initialization from powerflow ---"
      "\nVoltage across: {:s}"
      "\nCurrent: {:s}"
      "\nTerminal 0 voltage: {:s}"
      "\nTerminal 1 voltage: {:s}"
      "\nVirtual Node 1 voltage: {:s}"
      "\n--- Initialization from powerflow finished ---",
      Logger::phasorToString((**mIntfVoltage)(0, 0)),
      Logger::phasorToString((**mIntfCurrent)(0, 0)),
      Logger::phasorToString(initialSingleVoltage(0)),
      Logger::phasorToString(initialSingleVoltage(1)),
      Logger::phasorToString(mUseSubcomponents
                                 ? mVirtualNodes[0]->initialSingleVoltage()
                                 : Complex(0., 0.)));
}

void DP::Ph1::PiLine::mnaParentInitialize(Real omega, Real timeStep,
                                          Attribute<Matrix>::Ptr leftVector) {
  if (mUseSubcomponents)
    return;
  mDirectSeriesCond = MatrixComp::Zero(mNumFreqs, 1);
  mDirectInductorCond = MatrixComp::Zero(mNumFreqs, 1);
  mDirectSeriesPrevCurrentFactor = MatrixComp::Zero(mNumFreqs, 1);
  mDirectSeriesCurrent = MatrixComp::Zero(mNumFreqs, 1);
  mDirectSeriesVoltage = MatrixComp::Zero(mNumFreqs, 1);
  mDirectSeriesHistory = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapCond = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapPrevVoltageFactor = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapCurrent0 = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapCurrent1 = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapVoltage0 = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapVoltage1 = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapHistory0 = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCapHistory1 = MatrixComp::Zero(mNumFreqs, 1);

  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Real harmonicOmega = 2. * PI * mFrequencies(freq, 0);
    const Real b = timeStep * harmonicOmega / 2.;
    const Real a = timeStep / (2. * **mSeriesInd);
    mDirectInductorCond(freq, 0) =
        Complex(a / (1. + b * b), -a * b / (1. + b * b));
    mDirectSeriesCond(freq, 0) = mDirectInductorCond(freq, 0);
    mDirectSeriesPrevCurrentFactor(freq, 0) =
        Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
    const Real capG = 2. * mDirectShuntCapacitance / timeStep;
    const Real capB = harmonicOmega * mDirectShuntCapacitance;
    mDirectCapCond(freq, 0) = Complex(capG, capB);
    mDirectCapPrevVoltageFactor(freq, 0) = Complex(capG, -capB);
    if (freq == 0) {
      const Complex voltage1 = initialSingleVoltage(1);
      const Complex seriesCurrent = (**mIntfCurrent)(0, 0);
      mDirectSeriesCurrent(freq, 0) = seriesCurrent;
      mDirectSeriesVoltage(freq, 0) =
          (**mIntfVoltage)(0, 0) - **mSeriesRes * seriesCurrent;
      mDirectCapCurrent0(freq, 0) =
          Complex(0., harmonicOmega * mDirectShuntCapacitance) *
          initialSingleVoltage(0);
      mDirectCapCurrent1(freq, 0) =
          Complex(0., harmonicOmega * mDirectShuntCapacitance) * voltage1;
      mDirectCapVoltage0(freq, 0) = initialSingleVoltage(0);
      mDirectCapVoltage1(freq, 0) = voltage1;
    }
  }
}

void DP::Ph1::PiLine::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  const UInt terminal0 = matrixNodeIndex(0);
  const UInt terminal1 = matrixNodeIndex(1);
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    MNAStampUtils::stampAdmittance(
        Complex(mDirectSeriesConductance, 0.), systemMatrix, terminal0,
        virtualNode(0)->matrixNodeIndex(), terminalNotGrounded(0), true, mSLog,
        mNumFreqs, freq);
    MNAStampUtils::stampAdmittance(mDirectSeriesCond(freq, 0), systemMatrix,
                                   virtualNode(0)->matrixNodeIndex(), terminal1,
                                   true, terminalNotGrounded(1), mSLog,
                                   mNumFreqs, freq);
    MNAStampUtils::stampAdmittance(
        Complex(mDirectShuntConductance, 0.) + mDirectCapCond(freq, 0),
        systemMatrix, terminal0, 0, terminalNotGrounded(0), false, mSLog,
        mNumFreqs, freq);
    MNAStampUtils::stampAdmittance(
        Complex(mDirectShuntConductance, 0.) + mDirectCapCond(freq, 0),
        systemMatrix, terminal1, 0, terminalNotGrounded(1), false, mSLog,
        mNumFreqs, freq);
  }
}

void DP::Ph1::PiLine::mnaParentApplyRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents)
    return;
  const UInt terminal0 = matrixNodeIndex(0);
  const UInt terminal1 = matrixNodeIndex(1);
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    mDirectSeriesHistory(freq, 0) =
        mDirectInductorCond(freq, 0) * mDirectSeriesVoltage(freq, 0) +
        mDirectSeriesPrevCurrentFactor(freq, 0) * mDirectSeriesCurrent(freq, 0);
    mDirectCapHistory0(freq, 0) =
        -mDirectCapCurrent0(freq, 0) -
        mDirectCapPrevVoltageFactor(freq, 0) * mDirectCapVoltage0(freq, 0);
    mDirectCapHistory1(freq, 0) =
        -mDirectCapCurrent1(freq, 0) -
        mDirectCapPrevVoltageFactor(freq, 0) * mDirectCapVoltage1(freq, 0);
    Math::setVectorElement(rightVector, virtualNode(0)->matrixNodeIndex(),
                           mDirectSeriesHistory(freq, 0), mNumFreqs, freq);
    if (terminalNotGrounded(1))
      Math::setVectorElement(rightVector, terminal1,
                             -mDirectSeriesHistory(freq, 0) -
                                 mDirectCapHistory1(freq, 0),
                             mNumFreqs, freq);
  }
}

void DP::Ph1::PiLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  // add pre-step dependencies of component itself
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::PiLine::mnaParentPreStep(Real time, Int timeStepCount) {
  // pre-step of component itself
  this->mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::PiLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  // add post-step dependencies of component itself
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::PiLine::mnaParentPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  if (mUseSubcomponents) {
    this->mnaUpdateVoltage(**leftVector);
    this->mnaUpdateCurrent(**leftVector);
    return;
  }

  const Matrix &solution = **leftVector;
  const Bool terminal0Connected = terminalNotGrounded(0);
  const Bool terminal1Connected = terminalNotGrounded(1);
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Complex voltage0 =
        terminal0Connected ? Math::complexFromVectorElement(
                                 solution, matrixNodeIndex(0), mNumFreqs, freq)
                           : Complex(0., 0.);
    const Complex voltage1 =
        terminal1Connected ? Math::complexFromVectorElement(
                                 solution, matrixNodeIndex(1), mNumFreqs, freq)
                           : Complex(0., 0.);
    const Complex internalVoltage = Math::complexFromVectorElement(
        solution, virtualNode(0)->matrixNodeIndex(), mNumFreqs, freq);

    (**mIntfVoltage)(0, freq) = voltage1 - voltage0;
    mDirectSeriesVoltage(freq, 0) = voltage1 - internalVoltage;
    mDirectSeriesCurrent(freq, 0) =
        mDirectSeriesCond(freq, 0) * mDirectSeriesVoltage(freq, 0) +
        mDirectSeriesHistory(freq, 0);
    (**mIntfCurrent)(0, freq) = mDirectSeriesCurrent(freq, 0);

    mDirectCapVoltage0(freq, 0) = voltage0;
    mDirectCapVoltage1(freq, 0) = voltage1;
    mDirectCapCurrent0(freq, 0) =
        mDirectCapCond(freq, 0) * voltage0 + mDirectCapHistory0(freq, 0);
    mDirectCapCurrent1(freq, 0) =
        mDirectCapCond(freq, 0) * voltage1 + mDirectCapHistory1(freq, 0);
  }
}

void DP::Ph1::PiLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    (**mIntfVoltage)(0, freq) = 0.;
    if (terminalNotGrounded(1))
      (**mIntfVoltage)(0, freq) = Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(1), mNumFreqs, freq);
    if (terminalNotGrounded(0))
      (**mIntfVoltage)(0, freq) -= Math::complexFromVectorElement(
          leftVector, matrixNodeIndex(0), mNumFreqs, freq);
  }
}

void DP::Ph1::PiLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mUseSubcomponents) {
    (**mIntfCurrent)(0, 0) = mSubSeriesInductor->intfCurrent()(0, 0);
    return;
  }
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Complex internalVoltage = Math::complexFromVectorElement(
        leftVector, virtualNode(0)->matrixNodeIndex(), mNumFreqs, freq);
    const Complex terminalVoltage1 =
        terminalNotGrounded(1)
            ? Math::complexFromVectorElement(leftVector, matrixNodeIndex(1),
                                             mNumFreqs, freq)
            : Complex(0., 0.);
    mDirectSeriesVoltage(freq, 0) = terminalVoltage1 - internalVoltage;
    mDirectSeriesCurrent(freq, 0) =
        mDirectSeriesCond(freq, 0) * mDirectSeriesVoltage(freq, 0) +
        mDirectSeriesHistory(freq, 0);
    (**mIntfCurrent)(0, freq) = mDirectSeriesCurrent(freq, 0);
    const Complex voltage0 =
        terminalNotGrounded(0)
            ? Math::complexFromVectorElement(leftVector, matrixNodeIndex(0),
                                             mNumFreqs, freq)
            : Complex(0., 0.);
    const Complex voltageTerminal1 =
        terminalNotGrounded(1)
            ? Math::complexFromVectorElement(leftVector, matrixNodeIndex(1),
                                             mNumFreqs, freq)
            : Complex(0., 0.);
    mDirectCapVoltage0(freq, 0) = voltage0;
    mDirectCapVoltage1(freq, 0) = voltageTerminal1;
    mDirectCapCurrent0(freq, 0) =
        mDirectCapCond(freq, 0) * voltage0 + mDirectCapHistory0(freq, 0);
    mDirectCapCurrent1(freq, 0) = mDirectCapCond(freq, 0) * voltageTerminal1 +
                                  mDirectCapHistory1(freq, 0);
  }
}

// #### Tear Methods ####
MNAInterface::List DP::Ph1::PiLine::mnaTearGroundComponents() {
  MNAInterface::List gndComponents;

  if (!mUseSubcomponents)
    return gndComponents;

  gndComponents.push_back(mSubParallelResistor0);
  gndComponents.push_back(mSubParallelResistor1);

  if (**mParallelCap > 0) {
    gndComponents.push_back(mSubParallelCapacitor0);
    gndComponents.push_back(mSubParallelCapacitor1);
  }

  return gndComponents;
}

void DP::Ph1::PiLine::mnaTearInitialize(Real omega, Real timeStep) {
  if (!mUseSubcomponents)
    return;
  mSubSeriesResistor->mnaTearSetIdx(mTearIdx);
  mSubSeriesResistor->mnaTearInitialize(omega, timeStep);
  mSubSeriesInductor->mnaTearSetIdx(mTearIdx);
  mSubSeriesInductor->mnaTearInitialize(omega, timeStep);
}

void DP::Ph1::PiLine::mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) {
  if (!mUseSubcomponents)
    return;
  mSubSeriesResistor->mnaTearApplyMatrixStamp(tearMatrix);
  mSubSeriesInductor->mnaTearApplyMatrixStamp(tearMatrix);
}

void DP::Ph1::PiLine::mnaTearApplyVoltageStamp(Matrix &voltageVector) {
  if (!mUseSubcomponents)
    return;
  mSubSeriesInductor->mnaTearApplyVoltageStamp(voltageVector);
}

void DP::Ph1::PiLine::mnaTearPostStep(Complex voltage, Complex current) {
  if (!mUseSubcomponents) {
    mDirectSeriesVoltage(0, 0) = voltage - current * **mSeriesRes;
    mDirectSeriesCurrent(0, 0) = current;
    (**mIntfCurrent)(0, 0) = current;
    return;
  }
  mSubSeriesInductor->mnaTearPostStep(voltage - current * **mSeriesRes,
                                      current);
  (**mIntfCurrent)(0, 0) = mSubSeriesInductor->intfCurrent()(0, 0);
}
