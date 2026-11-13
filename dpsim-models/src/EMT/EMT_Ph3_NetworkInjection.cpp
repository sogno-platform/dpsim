/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_NetworkInjection.h>
#include <dpsim-models/Signal/CosineFMGenerator.h>
#include <dpsim-models/Signal/FrequencyRampGenerator.h>
#include <dpsim-models/Signal/SineWaveGenerator.h>

using namespace CPS;

EMT::Ph3::NetworkInjection::NetworkInjection(String uid, String name,
                                             Logger::Level logLevel)
    : NetworkInjection(uid, name, true, logLevel) {}

EMT::Ph3::NetworkInjection::NetworkInjection(String uid, String name,
                                             Bool useSubcomponents,
                                             Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mVoltageRef(mAttributes->createDynamic<MatrixComp>("V_ref")),
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
  mUseSubcomponents = useSubcomponents;
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(mUseSubcomponents ? 0 : 1);
  setTerminalNumber(1);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);

  // Create electrical sub components
  if (mUseSubcomponents) {
    mSubVoltageSource =
        std::make_shared<EMT::Ph3::VoltageSource>(**mName + "_vs", mLogLevel);
    addMNASubComponent(mSubVoltageSource,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
    mSubVoltageSource->mVoltageRef->setReference(mVoltageRef);
    mSubVoltageSource->mSrcFreq->setReference(mSrcFreq);
  }
}

SimPowerComp<Real>::Ptr EMT::Ph3::NetworkInjection::clone(String name) {
  auto copy = NetworkInjection::make(name, mUseSubcomponents, mLogLevel);
  copy->setParameters(**mVoltageRef);
  return copy;
}

void EMT::Ph3::NetworkInjection::setParameters(MatrixComp voltageRef,
                                               Real srcFreq) {
  mParametersSet = true;

  if (mUseSubcomponents)
    mSubVoltageSource->setParameters(voltageRef, srcFreq);
  else {
    auto source = Signal::SineWaveGenerator::make(**mName + "_sw");
    source->mFreq->setReference(mSrcFreq);
    source->setParameters(Complex(1, 0), srcFreq);
    mSrcSig = source;
    **mVoltageRef = voltageRef;
    **mSrcFreq = srcFreq;
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nVoltage Ref={:s} [V]"
                     "\nFrequency={:s} [Hz]",
                     Logger::matrixCompToString(voltageRef),
                     Logger::realToString(srcFreq));
}

void EMT::Ph3::NetworkInjection::setParameters(MatrixComp voltageRef,
                                               Real freqStart, Real rocof,
                                               Real timeStart, Real duration,
                                               bool smoothRamp) {
  mParametersSet = true;

  if (mUseSubcomponents)
    mSubVoltageSource->setParameters(voltageRef, freqStart, rocof, timeStart,
                                     duration, smoothRamp);
  else {
    auto source = Signal::FrequencyRampGenerator::make(**mName + "_fr");
    source->mFreq->setReference(mSrcFreq);
    source->setParameters(Complex(1, 0), freqStart, rocof, timeStart, duration,
                          smoothRamp);
    mSrcSig = source;
    **mVoltageRef = voltageRef;
    **mSrcFreq = freqStart;
  }

  SPDLOG_LOGGER_INFO(
      mSLog,
      "\nVoltage Ref={:s} [V]"
      "\nInitial frequency={:s} [Hz]"
      "\nRamp ROCOF={:s} [Hz/s]"
      "\nRamp duration={:s} [s]"
      "\nRamp nadir={:s} [Hz]",
      Logger::matrixCompToString(voltageRef), Logger::realToString(freqStart),
      Logger::realToString(rocof), Logger::realToString(duration),
      Logger::realToString(freqStart + rocof * duration));
}

void EMT::Ph3::NetworkInjection::setParameters(MatrixComp voltageRef,
                                               Real modulationFrequency,
                                               Real modulationAmplitude,
                                               Real baseFrequency /*= 0.0*/,
                                               bool zigzag /*= false*/) {
  mParametersSet = true;

  if (mUseSubcomponents)
    mSubVoltageSource->setParameters(voltageRef, modulationFrequency,
                                     modulationAmplitude, baseFrequency,
                                     zigzag);
  else {
    auto source = Signal::CosineFMGenerator::make(**mName + "_fm");
    source->mFreq->setReference(mSrcFreq);
    source->setParameters(Complex(1, 0), modulationFrequency,
                          modulationAmplitude, baseFrequency, zigzag);
    mSrcSig = source;
    **mVoltageRef = voltageRef;
    **mSrcFreq = baseFrequency;
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nVoltage Ref={:s} [V]"
                     "\nFrequency={:s} [Hz]",
                     Logger::matrixCompToString(voltageRef),
                     Logger::realToString(baseFrequency));
}

void EMT::Ph3::NetworkInjection::initializeParentFromNodesAndTerminals(
    Real frequency) {
  if (!mUseSubcomponents) {
    if (!mParametersSet) {
      auto source = Signal::SineWaveGenerator::make(**mName + "_sw");
      source->mFreq->setReference(mSrcFreq);
      source->setParameters(Complex(1, 0), frequency);
      mSrcSig = source;
      **mVoltageRef = CPS::Math::singlePhaseVariableToThreePhase(
          initialSingleVoltage(1) - initialSingleVoltage(0));
      **mSrcFreq = frequency;
    }
    return;
  }
  // Connect electrical subcomponents
  mSubVoltageSource->connect({SimNode::GND, node(0)});
}

// #### MNA functions ####
void EMT::Ph3::NetworkInjection::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!mUseSubcomponents) {
    for (UInt phase = 0; phase < 3; ++phase)
      Math::setVectorElement(
          rightVector,
          mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase)),
          (**mIntfVoltage)(phase, 0));
    return;
  }
  SPDLOG_LOGGER_DEBUG(mSLog, "Right Side Vector: {:s}",
                      Logger::matrixToString(rightVector));
}

void EMT::Ph3::NetworkInjection::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  for (UInt phase = 0; phase < 3; ++phase) {
    const UInt sourceCurrent =
        mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase));
    if (terminalNotGrounded(0)) {
      Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, phase),
                               sourceCurrent, 1.);
      Math::addToMatrixElement(systemMatrix, sourceCurrent,
                               matrixNodeIndex(0, phase), 1.);
    }
  }
}

Bool EMT::Ph3::NetworkInjection::mnaHasCompactRightSideVectorStamp() const {
  return !mUseSubcomponents ||
         CompositePowerComp<Real>::mnaHasCompactRightSideVectorStamp();
}

void EMT::Ph3::NetworkInjection::mnaAddCompactRightSideVectorStamp(
    Matrix &rightVector) {
  if (mUseSubcomponents) {
    CompositePowerComp<Real>::mnaAddCompactRightSideVectorStamp(rightVector);
    return;
  }
  for (UInt phase = 0; phase < 3; ++phase) {
    const UInt index =
        mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase));
    Math::addToVectorElement(
        rightVector, index, Math::realFromVectorElement(**mRightVector, index));
  }
}

void EMT::Ph3::NetworkInjection::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  if (!mUseSubcomponents) {
    attributeDependencies.push_back(mVoltageRef);
    attributeDependencies.push_back(mSrcFreq);
  }
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::NetworkInjection::mnaParentPreStep(Real time,
                                                  Int timeStepCount) {
  if (!mUseSubcomponents) {
    if (mSrcSig != nullptr) {
      mSrcSig->step(time);
      for (UInt phase = 0; phase < 3; ++phase)
        (**mIntfVoltage)(phase, 0) =
            RMS3PH_TO_PEAK1PH * Math::abs((**mVoltageRef)(phase, 0)) *
            cos(Math::phase(mSrcSig->getSignal()) +
                Math::phase((**mVoltageRef)(phase, 0)));
    } else
      **mIntfVoltage = RMS3PH_TO_PEAK1PH * (**mVoltageRef).real();
  }
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::NetworkInjection::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::NetworkInjection::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::NetworkInjection::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  if (mUseSubcomponents)
    **mIntfVoltage = **mSubVoltageSource->mIntfVoltage;
}

void EMT::Ph3::NetworkInjection::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  if (mUseSubcomponents)
    **mIntfCurrent = **mSubVoltageSource->mIntfCurrent;
  else
    for (UInt phase = 0; phase < 3; ++phase)
      (**mIntfCurrent)(phase, 0) = Math::realFromVectorElement(
          leftVector,
          mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase)));
}
