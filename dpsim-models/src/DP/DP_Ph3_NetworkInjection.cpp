// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_NetworkInjection.h>

using namespace CPS;

DP::Ph3::NetworkInjection::NetworkInjection(String uid, String name,
                                            Logger::Level logLevel)
    : NetworkInjection(uid, name, true, logLevel) {}

DP::Ph3::NetworkInjection::NetworkInjection(String uid, String name,
                                            Bool useSubcomponents,
                                            Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mUseSubcomponents(useSubcomponents),
      mVoltageRef(mAttributes->createDynamic<MatrixComp>("V_ref")),
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(mUseSubcomponents ? 0 : 1);
  setTerminalNumber(1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);

  // Create electrical sub components
  if (mUseSubcomponents) {
    mSubVoltageSource =
        std::make_shared<DP::Ph3::VoltageSource>(**mName + "_vs", mLogLevel);
    addMNASubComponent(mSubVoltageSource,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
    mSubVoltageSource->mVoltageRef->setReference(mVoltageRef);
    mSubVoltageSource->mSrcFreq->setReference(mSrcFreq);
  }
}

SimPowerComp<Complex>::Ptr DP::Ph3::NetworkInjection::clone(String name) {
  auto copy = NetworkInjection::make(name, mUseSubcomponents, mLogLevel);
  copy->setParameters(**mVoltageRef);
  return copy;
}

void DP::Ph3::NetworkInjection::setParameters(MatrixComp voltageRef,
                                              Real srcFreq) {
  mParametersSet = true;

  if (mUseSubcomponents)
    mSubVoltageSource->setParameters(voltageRef, srcFreq);
  else {
    **mVoltageRef = voltageRef;
    **mSrcFreq = srcFreq;
    auto signal = Signal::SineWaveGenerator::make(**mName + "_sw");
    signal->setParameters(Complex(1., 0.), srcFreq);
    mDirectSignal = signal;
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nVoltage Ref={:s} [V]"
                     "\nFrequency={:s} [Hz]",
                     Logger::matrixCompToString(voltageRef),
                     Logger::realToString(srcFreq));
}

void DP::Ph3::NetworkInjection::initializeParentFromNodesAndTerminals(
    Real frequency) {
  // Connect electrical subcomponents
  if (mUseSubcomponents) {
    mSubVoltageSource->connect({SimNode::GND, node(0)});
    return;
  }
  if ((**mVoltageRef).isZero())
    **mVoltageRef = Math::singlePhaseVariableToThreePhase(
        RMS3PH_TO_PEAK1PH * initialSingleVoltage(0));
  if (!mDirectSignal) {
    auto signal = Signal::SineWaveGenerator::make(**mName + "_sw");
    signal->setParameters(Complex(1., 0.), **mSrcFreq);
    mDirectSignal = signal;
  }
}

void DP::Ph3::NetworkInjection::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (!mUseSubcomponents) {
    updateMatrixNodeIndices();
  }
}

void DP::Ph3::NetworkInjection::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  const PhaseType phases[] = {PhaseType::A, PhaseType::B, PhaseType::C};
  for (UInt phase = 0; phase < 3; ++phase) {
    const UInt sourceCurrent = virtualNode(0)->matrixNodeIndex(phases[phase]);
    if (terminalNotGrounded(0)) {
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(0, phase),
                             sourceCurrent, Complex(1., 0.));
      Math::setMatrixElement(systemMatrix, sourceCurrent,
                             matrixNodeIndex(0, phase), Complex(1., 0.));
    }
  }
}

// #### MNA functions ####
void DP::Ph3::NetworkInjection::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!mUseSubcomponents) {
    const PhaseType phases[] = {PhaseType::A, PhaseType::B, PhaseType::C};
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(rightVector,
                             virtualNode(0)->matrixNodeIndex(phases[phase]),
                             (**mIntfVoltage)(phase, 0));
  }
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
  if (!mUseSubcomponents) {
    mDirectSignal->step(time);
    **mIntfVoltage = **mVoltageRef * mDirectSignal->getSignal();
  }
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
  if (mUseSubcomponents)
    **mIntfVoltage = **mSubVoltageSource->mIntfVoltage;
}

void DP::Ph3::NetworkInjection::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mUseSubcomponents)
    **mIntfCurrent = **mSubVoltageSource->mIntfCurrent;
  else
    for (UInt phase = 0; phase < 3; ++phase) {
      const PhaseType phases[] = {PhaseType::A, PhaseType::B, PhaseType::C};
      (**mIntfCurrent)(phase, 0) = Math::complexFromVectorElement(
          leftVector, virtualNode(0)->matrixNodeIndex(phases[phase]));
    }
}
