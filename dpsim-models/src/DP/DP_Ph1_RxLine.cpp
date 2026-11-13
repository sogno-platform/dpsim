/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_RxLine.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

DP::Ph1::RxLine::RxLine(String uid, String name, Logger::Level logLevel)
    : RxLine(uid, name, true, logLevel) {}

DP::Ph1::RxLine::RxLine(String uid, String name, Bool useSubcomponents,
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

///DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr DP::Ph1::RxLine::clone(String name) {
  auto copy = RxLine::make(name, mUseSubcomponents, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd);
  return copy;
}

void DP::Ph1::RxLine::createSubComponents() {
  if (!mUseSubcomponents)
    return;
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  mSubResistor =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
  mSubResistor->setParameters(**mSeriesRes);
  mSubResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubInductor =
      std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(**mSeriesInd);
  mSubInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mInitialResistor =
      std::make_shared<DP::Ph1::Resistor>(**mName + "_snubber_res", mLogLevel);
  mInitialResistor->setParameters(1e6);
  mInitialResistor->connect({SimNode::GND, mTerminals[1]->node()});
  addMNASubComponent(mInitialResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
}

void DP::Ph1::RxLine::initializeParentFromNodesAndTerminals(Real frequency) {
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = 0;
  mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) +
                                      (**mIntfCurrent)(0, 0) * **mSeriesRes);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

void DP::Ph1::RxLine::mnaParentInitialize(Real omega, Real timeStep,
                                          Attribute<Matrix>::Ptr leftVector) {
  if (mUseSubcomponents)
    return;
  mDirectInductorCond = MatrixComp::Zero(mNumFreqs, 1);
  mDirectPrevCurrentFactor = MatrixComp::Zero(mNumFreqs, 1);
  mDirectCurrent = MatrixComp::Zero(mNumFreqs, 1);
  mDirectVoltage = MatrixComp::Zero(mNumFreqs, 1);
  mDirectHistory = MatrixComp::Zero(mNumFreqs, 1);
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Real harmonicOmega = 2. * PI * mFrequencies(freq, 0);
    const Real b = timeStep * harmonicOmega / 2.;
    const Real a = timeStep / (2. * **mSeriesInd);
    mDirectInductorCond(freq, 0) =
        Complex(a / (1. + b * b), -a * b / (1. + b * b));
    mDirectPrevCurrentFactor(freq, 0) =
        Complex((1. - b * b) / (1. + b * b), -2. * b / (1. + b * b));
    if (freq == 0) {
      mDirectVoltage(freq, 0) =
          initialSingleVoltage(1) - mVirtualNodes[0]->initialSingleVoltage();
      const Complex initialCurrent =
          harmonicOmega != 0. ? mDirectVoltage(freq, 0) /
                                    Complex(0., harmonicOmega * **mSeriesInd)
                              : Complex(0., 0.);
      // Match the DP Inductor's initialization recurrence exactly: initVars()
      // stores G*v + K*i and then initializes i to G*v + that history.
      mDirectCurrent(freq, 0) =
          2. * mDirectInductorCond(freq, 0) * mDirectVoltage(freq, 0) +
          mDirectPrevCurrentFactor(freq, 0) * initialCurrent;
    }
  }
}

void DP::Ph1::RxLine::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  const UInt terminal0 = matrixNodeIndex(0);
  const UInt terminal1 = matrixNodeIndex(1);
  const UInt internal = virtualNode(0)->matrixNodeIndex();
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    MNAStampUtils::stampAdmittance(Complex(1. / **mSeriesRes, 0.), systemMatrix,
                                   terminal0, internal, terminalNotGrounded(0),
                                   true, mSLog, mNumFreqs, freq);
    MNAStampUtils::stampAdmittance(
        mDirectInductorCond(freq, 0), systemMatrix, internal, terminal1, true,
        terminalNotGrounded(1), mSLog, mNumFreqs, freq);
    MNAStampUtils::stampAdmittance(Complex(1e-6, 0.), systemMatrix, terminal1,
                                   0, terminalNotGrounded(1), false, mSLog,
                                   mNumFreqs, freq);
  }
}

void DP::Ph1::RxLine::mnaParentApplyRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents)
    return;
  const UInt terminal1 = matrixNodeIndex(1);
  const UInt internal = virtualNode(0)->matrixNodeIndex();
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    mDirectHistory(freq, 0) =
        mDirectInductorCond(freq, 0) * mDirectVoltage(freq, 0) +
        mDirectPrevCurrentFactor(freq, 0) * mDirectCurrent(freq, 0);
    Math::setVectorElement(rightVector, internal, mDirectHistory(freq, 0),
                           mNumFreqs, freq);
    if (terminalNotGrounded(1))
      Math::setVectorElement(rightVector, terminal1, -mDirectHistory(freq, 0),
                             mNumFreqs, freq);
  }
}

void DP::Ph1::RxLine::mnaParentPreStep(Real time, Int timeStepCount) {
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::RxLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::RxLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph1::RxLine::mnaParentPostStep(Real time, Int timeStepCount,
                                        Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph1::RxLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::RxLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mUseSubcomponents) {
    (**mIntfCurrent)(0, 0) = mSubInductor->intfCurrent()(0, 0);
    return;
  }
  const UInt internal = virtualNode(0)->matrixNodeIndex();
  for (UInt freq = 0; freq < mNumFreqs; ++freq) {
    const Complex internalVoltage =
        Math::complexFromVectorElement(leftVector, internal, mNumFreqs, freq);
    const Complex terminalVoltage =
        terminalNotGrounded(1)
            ? Math::complexFromVectorElement(leftVector, matrixNodeIndex(1),
                                             mNumFreqs, freq)
            : Complex(0., 0.);
    mDirectVoltage(freq, 0) = terminalVoltage - internalVoltage;
    mDirectCurrent(freq, 0) =
        mDirectInductorCond(freq, 0) * mDirectVoltage(freq, 0) +
        mDirectHistory(freq, 0);
    (**mIntfCurrent)(0, freq) = mDirectCurrent(freq, 0);
  }
}
