/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_VoltageSourceRamp.h>

using namespace CPS;

EMT::Ph1::VoltageSourceRamp::VoltageSourceRamp(String uid, String name,
                                               Logger::Level logLevel)
    : VoltageSourceRamp(uid, name, true, logLevel) {}

EMT::Ph1::VoltageSourceRamp::VoltageSourceRamp(String uid, String name,
                                               Bool useSubcomponents,
                                               Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, false, logLevel),
      mVoltageRef(mAttributes->create<Complex>("V_ref")),
      mSrcFreq(mAttributes->create<Real>("f_src")) {
  mUseSubcomponents = useSubcomponents;
  setVirtualNodeNumber(1);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

/// DEPRECATED: delete method
SimPowerComp<Real>::Ptr EMT::Ph1::VoltageSourceRamp::clone(String name) {
  auto copy = VoltageSourceRamp::make(name, mUseSubcomponents, mLogLevel);
  copy->setParameters(**mVoltageRef, mAddVoltage, **mSrcFreq, mAddSrcFreq,
                      mSwitchTime, mRampTime);
  return copy;
}

void EMT::Ph1::VoltageSourceRamp::setParameters(Complex voltage,
                                                Complex addVoltage,
                                                Real srcFreq, Real addSrcFreq,
                                                Real switchTime,
                                                Real rampTime) {
  **mVoltageRef = voltage;
  mAddVoltage = addVoltage;
  **mSrcFreq = srcFreq;
  mAddSrcFreq = addSrcFreq;
  mSwitchTime = switchTime;
  mRampTime = rampTime;

  mParametersSet = true;
}

void EMT::Ph1::VoltageSourceRamp::initialize(Matrix frequencies) {
  SimPowerComp<Real>::initialize(frequencies);

  if (**mVoltageRef == Complex(0, 0))
    **mVoltageRef =
        RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));

  if (mUseSubcomponents) {
    mSubVoltageSource = VoltageSource::make(**mName + "_src", mLogLevel);
    mSubVoltageSource->setParameters(**mVoltageRef, 0);
    mSubVoltageSource->connect({node(0), node(1)});
    mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
    mSubVoltageSource->initialize(frequencies);
    addMNASubComponent(mSubVoltageSource,
                       MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT,
                       MNA_SUBCOMP_TASK_ORDER::NO_TASK, true);
  }
}

void EMT::Ph1::VoltageSourceRamp::updateState(Real time) {
  Real voltageAbs = Math::abs(**mVoltageRef);
  Real voltagePhase = Math::phase(**mVoltageRef);
  (**mIntfVoltage)(0, 0) = voltageAbs * cos(voltagePhase + **mSrcFreq * time);

  if (time >= mSwitchTime && time < mSwitchTime + mRampTime) {
    voltageAbs = Math::abs(**mVoltageRef +
                           (time - mSwitchTime) / mRampTime * mAddVoltage);
    voltagePhase = Math::phase(**mVoltageRef +
                               (time - mSwitchTime) / mRampTime * mAddVoltage);
    Real fadeInOut =
        0.5 + 0.5 * sin((time - mSwitchTime) / mRampTime * PI + -PI / 2);
    (**mIntfVoltage)(0, 0) =
        voltageAbs *
        cos(voltagePhase + (**mSrcFreq + fadeInOut * mAddSrcFreq) * time);
  } else if (time >= mSwitchTime + mRampTime) {
    voltageAbs = Math::abs(**mVoltageRef + mAddVoltage);
    voltagePhase = Math::phase(**mVoltageRef + mAddVoltage);
    (**mIntfVoltage)(0, 0) =
        voltageAbs * cos(voltagePhase + (**mSrcFreq + mAddSrcFreq) * time);
  }
}

void EMT::Ph1::VoltageSourceRamp::mnaParentPreStep(Real time,
                                                   Int timeStepCount) {
  updateState(time);
  if (mUseSubcomponents)
    **mSubVoltageSource->mVoltageRef = (**mIntfVoltage)(0, 0);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
  if (!mUseSubcomponents)
    mDirectVoltageForRhs = (**mIntfVoltage)(0, 0);
}

void EMT::Ph1::VoltageSourceRamp::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  if (terminalNotGrounded(0)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0),
                             mVirtualNodes[0]->matrixNodeIndex(), -1.);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                             matrixNodeIndex(0), -1.);
  }
  if (terminalNotGrounded(1)) {
    Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1),
                             mVirtualNodes[0]->matrixNodeIndex(), 1.);
    Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(),
                             matrixNodeIndex(1), 1.);
  }
}

void EMT::Ph1::VoltageSourceRamp::mnaParentApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (!mUseSubcomponents)
    Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(),
                           mDirectVoltageForRhs);
}

void EMT::Ph1::VoltageSourceRamp::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mVoltageRef);
  attributeDependencies.push_back(mSrcFreq);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph1::VoltageSourceRamp::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::VoltageSourceRamp::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  if (!mUseSubcomponents)
    (**mIntfCurrent)(0, 0) = Math::realFromVectorElement(
        **leftVector, mVirtualNodes[0]->matrixNodeIndex());
}
