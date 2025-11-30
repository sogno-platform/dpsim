/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_varResSwitch.h>

using namespace CPS;

DP::Ph1::varResSwitch::varResSwitch(String uid, String name,
                                    Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph1::Switch(mAttributes) {
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph1::varResSwitch::clone(String name) {
  auto copy = varResSwitch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  return copy;
}

void DP::Ph1::varResSwitch::initializeFromNodesAndTerminals(Real frequency) {

  // // This function is not used!!!!!!

  //Switch Resistance
  Real impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;

  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;
}

// #### MNA functions ####
void DP::Ph1::varResSwitch::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
}

void DP::Ph1::varResSwitch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  Complex conductance = (**mIsClosed) ? Complex(1. / **mClosedResistance, 0)
                                      : Complex(1. / **mOpenResistance, 0);

  MNAStampUtils::stampAdmittance(conductance, systemMatrix, matrixNodeIndex(0),
                                 matrixNodeIndex(1), terminalNotGrounded(0),
                                 terminalNotGrounded(1), mSLog);
}

void DP::Ph1::varResSwitch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  Complex conductance = (closed) ? Complex(1. / **mClosedResistance, 0)
                                 : Complex(1. / **mOpenResistance, 0);

  MNAStampUtils::stampAdmittance(conductance, systemMatrix, matrixNodeIndex(0),
                                 matrixNodeIndex(1), terminalNotGrounded(0),
                                 terminalNotGrounded(1), mSLog);
}

void DP::Ph1::varResSwitch::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {}

Bool DP::Ph1::varResSwitch::mnaIsClosed() { return isClosed(); }

void DP::Ph1::varResSwitch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {

  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::varResSwitch::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph1::varResSwitch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::varResSwitch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = (**mIsClosed)
                               ? (**mIntfVoltage)(0, 0) / **mClosedResistance
                               : (**mIntfVoltage)(0, 0) / **mOpenResistance;
}

Bool DP::Ph1::varResSwitch::hasParameterChanged() {
  // Check if state of switch changed from open to closed
  if (!(mIsClosedPrev == this->mnaIsClosed())) {
    // Switch is closed : change with 1/mDeltaRes
    if (this->mnaIsClosed() == true) {
      // mClosedResistance= 1./mDeltaRes*mPrevRes;
      **mClosedResistance = mDeltaResClosed * mPrevRes;
      mPrevRes = **mClosedResistance;
      // check if target value is reached
      if (**mClosedResistance < mInitClosedRes) {
        **mClosedResistance = mInitClosedRes;
        mPrevRes = **mClosedResistance;
        mIsClosedPrev = this->mnaIsClosed();
      }
    }
    // Switch is opened : change with mDeltaRes
    else if (this->mnaIsClosed() == false) {
      **mOpenResistance = mDeltaResOpen * mPrevRes;
      mPrevRes = **mOpenResistance;
      // check if target value is reached
      if (**mOpenResistance > mInitOpenRes) {
        **mOpenResistance = mInitOpenRes;
        mPrevRes = **mOpenResistance;
        mIsClosedPrev = this->mnaIsClosed();
      }
    }
    return 1; //recompute system matrix
  } else {
    return 0; // do not recompute system matrix
  }
}

void DP::Ph1::varResSwitch::setInitParameters(Real timestep) {
  //Define variables for the transition
  mDeltaResClosed = 0;
  // mDeltaResOpen = 1.5; // assumption for 1ms step size
  mDeltaResOpen = 0.5 * timestep / 0.001 + 1;
  mIsClosedPrev = **mIsClosed;
  mPrevRes = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  mInitClosedRes = **mClosedResistance;
  mInitOpenRes = **mOpenResistance;
}
