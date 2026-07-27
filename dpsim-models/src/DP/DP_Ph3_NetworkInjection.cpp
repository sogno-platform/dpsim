// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_NetworkInjection.h>

using namespace CPS;

DP::Ph3::NetworkInjection::NetworkInjection(String uid, String name,
                                            Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mVoltageRef(mAttributes->createDynamic<MatrixComp>("V_ref")),
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(0);
  setTerminalNumber(1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);

  // Create electrical sub components
  mSubVoltageSource =
      std::make_shared<DP::Ph3::VoltageSource>(**mName + "_vs", mLogLevel);
  addMNASubComponent(mSubVoltageSource,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  SPDLOG_LOGGER_INFO(mSLog, "Electrical subcomponents: ");
  for (auto subcomp : mSubComponents)
    SPDLOG_LOGGER_INFO(mSLog, "- {}", subcomp->name());

  mSubVoltageSource->mVoltageRef->setReference(mVoltageRef);
  mSubVoltageSource->mSrcFreq->setReference(mSrcFreq);
}

SimPowerComp<Complex>::Ptr DP::Ph3::NetworkInjection::clone(String name) {
  auto copy = NetworkInjection::make(name, mLogLevel);
  copy->setParameters(**mVoltageRef);
  return copy;
}

void DP::Ph3::NetworkInjection::setParameters(MatrixComp voltageRef,
                                              Real srcFreq) {
  mParametersSet = true;

  mSubVoltageSource->setParameters(voltageRef, srcFreq);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nVoltage Ref={:s} [V]"
                     "\nFrequency={:s} [Hz]",
                     Logger::matrixCompToString(voltageRef),
                     Logger::realToString(srcFreq));
}

void DP::Ph3::NetworkInjection::initializeParentFromNodesAndTerminals(
    Real frequency) {
  // Connect electrical subcomponents
  mSubVoltageSource->connect({SimNode::GND, node(0)});
}

// #### MNA functions ####
void DP::Ph3::NetworkInjection::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  SPDLOG_LOGGER_DEBUG(mSLog, "Right Side Vector: {:s}",
                      Logger::matrixToString(rightVector));
}

void DP::Ph3::NetworkInjection::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph3::NetworkInjection::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::NetworkInjection::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::NetworkInjection::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph3::NetworkInjection::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mIntfVoltage = **mSubVoltageSource->mIntfVoltage;
}

void DP::Ph3::NetworkInjection::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = **mSubVoltageSource->mIntfCurrent;
}
