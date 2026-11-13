/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_VoltageSourceRamp.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::VoltageSourceRamp::VoltageSourceRamp(String uid, String name,
                                              Logger::Level logLevel)
    : VoltageSourceRamp(uid, name, true, logLevel) {}

DP::Ph1::VoltageSourceRamp::VoltageSourceRamp(String uid, String name,
                                              Bool useSubcomponents,
                                              Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, false, logLevel),
      mUseSubcomponents(useSubcomponents),
      mVoltageRef(mAttributes->create<Complex>("V_ref")),
      mSrcFreq(mAttributes->create<Real>("f_src")) {
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

/// DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr DP::Ph1::VoltageSourceRamp::clone(String name) {
  auto copy = VoltageSourceRamp::make(name, mUseSubcomponents, mLogLevel);
  copy->setParameters(**mVoltageRef, mAddVoltage, **mSrcFreq, mAddSrcFreq,
                      mSwitchTime, mRampTime);
  return copy;
}

void DP::Ph1::VoltageSourceRamp::setParameters(Complex voltage,
                                               Complex addVoltage, Real srcFreq,
                                               Real addSrcFreq, Real switchTime,
                                               Real rampTime) {
  **mVoltageRef = voltage;
  mAddVoltage = addVoltage;
  **mSrcFreq = srcFreq;
  mAddSrcFreq = addSrcFreq;
  mSwitchTime = switchTime;
  mRampTime = rampTime;

  mParametersSet = true;
}

void DP::Ph1::VoltageSourceRamp::initialize(Matrix frequencies) {
  SimPowerComp<Complex>::initialize(frequencies);

  if (**mVoltageRef == Complex(0, 0))
    **mVoltageRef = initialSingleVoltage(1) - initialSingleVoltage(0);

  if (mUseSubcomponents) {
    mSubVoltageSource = VoltageSource::make(**mName + "_src", mLogLevel);
    mSubVoltageSource->setParameters(**mVoltageRef, **mSrcFreq);
    mSubVoltageSource->connect({node(0), node(1)});
    mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
    mSubVoltageSource->initialize(frequencies);
    addMNASubComponent(mSubVoltageSource,
                       MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::NO_TASK, true);
  }
}

void DP::Ph1::VoltageSourceRamp::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (!mUseSubcomponents)
    updateMatrixNodeIndices();
}

void DP::Ph1::VoltageSourceRamp::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  const UInt sourceCurrent = virtualNode(0)->matrixNodeIndex();
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    if (terminalNotGrounded(0)) {
      Math::setMatrixElement(systemMatrix, sourceCurrent, matrixNodeIndex(0),
                             Complex(-1., 0.), mNumFreqs, freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(0), sourceCurrent,
                             Complex(-1., 0.), mNumFreqs, freq);
    }
    if (terminalNotGrounded(1)) {
      Math::setMatrixElement(systemMatrix, sourceCurrent, matrixNodeIndex(1),
                             Complex(1., 0.), mNumFreqs, freq);
      Math::setMatrixElement(systemMatrix, matrixNodeIndex(1), sourceCurrent,
                             Complex(1., 0.), mNumFreqs, freq);
    }
  }
}

void DP::Ph1::VoltageSourceRamp::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!mUseSubcomponents)
    Math::setVectorElement(rightVector, virtualNode(0)->matrixNodeIndex(),
                           (**mIntfVoltage)(0, 0), mNumFreqs, 0);
}

void DP::Ph1::VoltageSourceRamp::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  if (!mUseSubcomponents) {
    attributeDependencies.push_back(leftVector);
    modifiedAttributes.push_back(mIntfCurrent);
  }
}

void DP::Ph1::VoltageSourceRamp::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  if (!mUseSubcomponents)
    (**mIntfCurrent)(0, 0) = Math::complexFromVectorElement(
        **leftVector, virtualNode(0)->matrixNodeIndex(), mNumFreqs, 0);
}

void DP::Ph1::VoltageSourceRamp::initializeParentFromNodesAndTerminals(
    Real frequency) {}

void DP::Ph1::VoltageSourceRamp::updateState(Real time) {
  (**mIntfVoltage)(0, 0) = **mVoltageRef;

  if (time >= mSwitchTime && time < mSwitchTime + mRampTime) {
    Real voltageAbs = Math::abs(**mVoltageRef +
                                (time - mSwitchTime) / mRampTime * mAddVoltage);
    Real voltagePhase = Math::phase(
        **mVoltageRef + (time - mSwitchTime) / mRampTime * mAddVoltage);
    Real fadeInOut =
        0.5 + 0.5 * sin((time - mSwitchTime) / mRampTime * PI + -PI / 2);
    (**mIntfVoltage)(0, 0) =
        Math::polar(voltageAbs, voltagePhase + fadeInOut * mAddSrcFreq * time);
  } else if (time >= mSwitchTime + mRampTime) {
    Real voltageAbs = Math::abs(**mVoltageRef + mAddVoltage);
    Real voltagePhase = Math::phase(**mVoltageRef + mAddVoltage);
    (**mIntfVoltage)(0, 0) =
        Math::polar(voltageAbs, voltagePhase + mAddSrcFreq * time);
  }
}

void DP::Ph1::VoltageSourceRamp::mnaParentPreStep(Real time,
                                                  Int timeStepCount) {
  updateState(time);
  if (mUseSubcomponents)
    **mSubVoltageSource->mVoltageRef = (**mIntfVoltage)(0, 0);
  else
    mnaParentApplyRightSideVectorStamp(**mRightVector);
}
