/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorIdeal.h>

using namespace CPS;

EMT::Ph3::SynchronGeneratorIdeal::SynchronGeneratorIdeal(
    String uid, String name, Logger::Level logLevel,
    CPS::GeneratorType sourceType)
    : SynchronGeneratorIdeal(uid, name, true, logLevel, sourceType) {}

EMT::Ph3::SynchronGeneratorIdeal::SynchronGeneratorIdeal(
    String uid, String name, Bool useSubcomponents, Logger::Level logLevel,
    CPS::GeneratorType sourceType)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mRefVoltage(mAttributes->createDynamic<MatrixComp>("V_ref")),
      mRefCurrent(mAttributes->createDynamic<MatrixComp>("I_ref")),
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
  mPhaseType = PhaseType::ABC;
  mSourceType = sourceType;
  mUseSubcomponents = useSubcomponents;

  if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
    setVirtualNodeNumber(1);
  else
    setVirtualNodeNumber(0);

  setTerminalNumber(1);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

EMT::Ph3::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String name,
                                                         Logger::Level logLevel)
    : SynchronGeneratorIdeal(name, name, logLevel) {}

SimPowerComp<Real>::Ptr EMT::Ph3::SynchronGeneratorIdeal::clone(String name) {
  return SynchronGeneratorIdeal::make(name, mUseSubcomponents, mLogLevel,
                                      mSourceType);
}

void EMT::Ph3::SynchronGeneratorIdeal::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  if (!mUseSubcomponents)
    return;

  if (mSourceType == CPS::GeneratorType::IdealVoltageSource) {
    mSubVoltageSource =
        EMT::Ph3::VoltageSource::make(**mName + "_vs", mLogLevel);
    addMNASubComponent(mSubVoltageSource,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  } else {
    mSubCurrentSource =
        EMT::Ph3::CurrentSource::make(**mName + "_cs", mLogLevel);
    addMNASubComponent(mSubCurrentSource,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  }

  mSubComponents[0]->connect({SimNode::GND, node(0)});

  if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
    mSubComponents[0]->setVirtualNodeAt(mVirtualNodes[0], 0);

  if (mSourceType == CPS::GeneratorType::IdealCurrentSource)
    mSubComponents[0]->setTerminalAt(terminal(0), 1);

  if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
    mRefVoltage->setReference(
        mSubComponents[0]->attributeTyped<MatrixComp>("V_ref"));
  if (mSourceType == CPS::GeneratorType::IdealCurrentSource)
    mRefCurrent->setReference(
        mSubComponents[0]->attributeTyped<MatrixComp>("I_ref"));
  mSrcFreq->setReference(mSubComponents[0]->attributeTyped<Real>("f_src"));
}

void EMT::Ph3::SynchronGeneratorIdeal::initializeParentFromNodesAndTerminals(
    Real frequency) {
  if (!mUseSubcomponents) {
    **mSrcFreq = frequency;
    if (mSourceType == CPS::GeneratorType::IdealVoltageSource) {
      **mRefVoltage =
          Math::singlePhaseVariableToThreePhase(initialSingleVoltage(0));
    } else {
      const Complex voltage = initialSingleVoltage(0);
      const Complex power = terminal(0)->singlePower();
      const Complex current = std::abs(voltage) > 0.
                                  ? std::conj(power / voltage / sqrt(3.))
                                  : Complex(0., 0.);
      **mRefCurrent = Math::singlePhaseVariableToThreePhase(current);
    }
  }
  // The voltage-source representation has its internal source node behind
  // the ideal source.  Seed that node from the solved terminal voltage so the
  // child VoltageSource can derive its reference phasor automatically.
  // Without this, the virtual node stays at zero and the child source starts
  // with a zero-voltage reference even though initWithPowerflow() populated
  // the external terminal.
  if (mSourceType == CPS::GeneratorType::IdealVoltageSource &&
      !mVirtualNodes.empty() && mVirtualNodes[0]) {
    mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0));
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 0 power: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::complexToString(terminal(0)->singlePower()));
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  if (!mUseSubcomponents) {
    if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
      attributeDependencies.push_back(mRefVoltage);
    else
      attributeDependencies.push_back(mRefCurrent);
    attributeDependencies.push_back(mSrcFreq);
    modifiedAttributes.push_back(mIntfVoltage);
    modifiedAttributes.push_back(mIntfCurrent);
    modifiedAttributes.push_back(mRightVector);
    return;
  }
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaParentPreStep(Real time,
                                                        Int timeStepCount) {
  if (!mUseSubcomponents) {
    if (mSourceType == CPS::GeneratorType::IdealVoltageSource) {
      for (Int phase = 0; phase < 3; ++phase) {
        const Complex reference = (**mRefVoltage)(phase, 0);
        (**mIntfVoltage)(phase, 0) =
            RMS3PH_TO_PEAK1PH * Math::abs(reference) *
            std::cos(2. * PI * **mSrcFreq * time + Math::phase(reference));
      }
    } else {
      for (Int phase = 0; phase < 3; ++phase) {
        const Complex reference = (**mRefCurrent)(phase, 0);
        (**mIntfCurrent)(phase, 0) =
            RMS_TO_PEAK * Math::abs(reference) *
            std::cos(2. * PI * **mSrcFreq * time + Math::phase(reference));
      }
    }
  }
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents ||
      mSourceType != CPS::GeneratorType::IdealVoltageSource ||
      !terminalNotGrounded(0))
    return;
  for (Int phase = 0; phase < 3; ++phase) {
    const UInt nodeIndex = matrixNodeIndex(0, phase);
    const UInt sourceIndex =
        mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase));
    Math::addToMatrixElement(systemMatrix, nodeIndex, sourceIndex, 1.);
    Math::addToMatrixElement(systemMatrix, sourceIndex, nodeIndex, 1.);
  }
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents)
    return;
  if (mSourceType == CPS::GeneratorType::IdealVoltageSource) {
    for (Int phase = 0; phase < 3; ++phase)
      Math::setVectorElement(
          rightVector,
          mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          (**mIntfVoltage)(phase, 0));
  } else if (terminalNotGrounded(0)) {
    for (Int phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector, matrixNodeIndex(0, phase),
                             -(**mIntfCurrent)(phase, 0));
  }
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaCompUpdateCurrent(
    const Matrix &leftvector) {
  if (mUseSubcomponents) {
    **mIntfCurrent = **mSubComponents[0]->mIntfCurrent;
    return;
  }
  if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
    for (Int phase = 0; phase < 3; ++phase)
      (**mIntfCurrent)(phase, 0) = Math::realFromVectorElement(
          leftvector,
          mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase)));
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  if (mUseSubcomponents) {
    **mIntfVoltage = **mSubComponents[0]->mIntfVoltage;
    return;
  }
  (**mIntfVoltage).setZero();
  if (terminalNotGrounded(0))
    for (Int phase = 0; phase < 3; ++phase)
      (**mIntfVoltage)(phase, 0) =
          Math::realFromVectorElement(leftVector, matrixNodeIndex(0, phase));
}
