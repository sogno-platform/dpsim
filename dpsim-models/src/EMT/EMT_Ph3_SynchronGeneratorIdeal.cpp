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
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mRefVoltage(mAttributes->createDynamic<MatrixComp>("V_ref")) {
  mPhaseType = PhaseType::ABC;
  mSourceType = sourceType;

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
  return SynchronGeneratorIdeal::make(name, mLogLevel);
}

void EMT::Ph3::SynchronGeneratorIdeal::initializeFromNodesAndTerminals(
    Real frequency) {

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

  mSubComponents[0]->initialize(mFrequencies);
  mSubComponents[0]->initializeFromNodesAndTerminals(frequency);

  if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
    mRefVoltage->setReference(
        mSubComponents[0]->attributeTyped<MatrixComp>("V_ref"));

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
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaParentPreStep(Real time,
                                                        Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
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
  **mIntfCurrent = **mSubComponents[0]->mIntfCurrent;
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  **mIntfVoltage = **mSubComponents[0]->mIntfVoltage;
}
