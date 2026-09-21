/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include <dpsim-models/Signal/DecouplingLine.h>

using namespace CPS;
using namespace CPS::DP::Ph1;
using namespace CPS::Signal;

DecouplingLine::DecouplingLine(String uid, String name, Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcCur1Ref(mAttributes->create<Complex>("i_src1")),
      mSrcCur2Ref(mAttributes->create<Complex>("i_src2")) {

  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

void DecouplingLine::setParameters(Real resistance, Real inductance,
                                   Real capacitance) {

  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;

  mSurgeImpedance = sqrt(inductance / capacitance);
  mDelay = sqrt(inductance * capacitance);
  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mParametersSet = true;
}

void DecouplingLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  mRes1 = Resistor::make(**mName + "_r1", mLogLevel);
  mRes1->setParameters(mSurgeImpedance + mResistance / 4);
  mRes1->connect({mTerminals[0]->node(), SimNode<Complex>::GND});
  addMNASubComponent(mRes1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mRes2 = Resistor::make(**mName + "_r2", mLogLevel);
  mRes2->setParameters(mSurgeImpedance + mResistance / 4);
  mRes2->connect({mTerminals[1]->node(), SimNode<Complex>::GND});
  addMNASubComponent(mRes2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSrc1 = CurrentSource::make(**mName + "_i1", mLogLevel);
  mSrc1->setParameters(0);
  mSrc1->connect({mTerminals[0]->node(), SimNode<Complex>::GND});
  addMNASubComponent(mSrc1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrc2 = CurrentSource::make(**mName + "_i2", mLogLevel);
  mSrc2->setParameters(0);
  mSrc2->connect({mTerminals[1]->node(), SimNode<Complex>::GND});
  addMNASubComponent(mSrc2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrcCur1 = mSrc1->mCurrentRef;
  mSrcCur2 = mSrc2->mCurrentRef;
}

void DecouplingLine::initializeParentFromNodesAndTerminals(Real frequency) {
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
}

void DecouplingLine::mnaParentInitialize(Real omega, Real timeStep,
                                         Attribute<Matrix>::Ptr leftVector) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  Complex volt1 = initialSingleVoltage(0);
  Complex volt2 = initialSingleVoltage(1);
  // TODO different initialization for lumped resistance?
  Complex initAdmittance = 1. / Complex(mResistance, omega * mInductance) +
                           Complex(0, omega * mCapacitance / 2);
  Complex cur1 = volt1 * initAdmittance -
                 volt2 / Complex(mResistance, omega * mInductance);
  Complex cur2 = volt2 * initAdmittance -
                 volt1 / Complex(mResistance, omega * mInductance);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  (**mIntfCurrent)(0, 0) = cur1;

  // Resize ring buffers and initialize
  mVolt1.resize(mBufSize, volt1);
  mVolt2.resize(mBufSize, volt2);
  mCur1.resize(mBufSize, cur1);
  mCur2.resize(mBufSize, cur2);
}

Complex DecouplingLine::interpolate(std::vector<Complex> &data) {
  // linear interpolation of the nearest values
  Complex c1 = data[mBufIdx];
  Complex c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

void DecouplingLine::step(Real time, Int timeStepCount) {
  Complex volt1 = interpolate(mVolt1);
  Complex volt2 = interpolate(mVolt2);
  Complex cur1 = interpolate(mCur1);
  Complex cur2 = interpolate(mCur2);

  if (timeStepCount == 0) {
    // bit of a hack for proper initialization
    **mSrcCur1Ref = cur1 - volt1 / (mSurgeImpedance + mResistance / 4);
    **mSrcCur2Ref = cur2 - volt2 / (mSurgeImpedance + mResistance / 4);
  } else {
    // Update currents
    Real denom = (mSurgeImpedance + mResistance / 4) *
                 (mSurgeImpedance + mResistance / 4);
    **mSrcCur1Ref = -mSurgeImpedance / denom *
                        (volt2 + (mSurgeImpedance - mResistance / 4) * cur2) -
                    mResistance / 4 / denom *
                        (volt1 + (mSurgeImpedance - mResistance / 4) * cur1);
    **mSrcCur2Ref = -mSurgeImpedance / denom *
                        (volt1 + (mSurgeImpedance - mResistance / 4) * cur1) -
                    mResistance / 4 / denom *
                        (volt2 + (mSurgeImpedance - mResistance / 4) * cur2);
    **mSrcCur1Ref = **mSrcCur1Ref * Complex(cos(-2. * PI * 50 * mDelay),
                                            sin(-2. * PI * 50 * mDelay));
    **mSrcCur2Ref = **mSrcCur2Ref * Complex(cos(-2. * PI * 50 * mDelay),
                                            sin(-2. * PI * 50 * mDelay));
  }
  mSrcCur1->set(**mSrcCur1Ref);
  mSrcCur2->set(**mSrcCur2Ref);
}

void DecouplingLine::postStep() {
  // Update ringbuffers with new values
  mVolt1[mBufIdx] = -mRes1->intfVoltage()(0, 0);
  mVolt2[mBufIdx] = -mRes2->intfVoltage()(0, 0);
  mCur1[mBufIdx] = -mRes1->intfCurrent()(0, 0) + mSrcCur1->get();
  mCur2[mBufIdx] = -mRes2->intfCurrent()(0, 0) + mSrcCur2->get();

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;
}

void DecouplingLine::mnaParentPreStep(Real time, Int timeStepCount) {
  step(time, timeStepCount);
  mSrc1->mnaPreStep(time, timeStepCount);
  mSrc2->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DecouplingLine::mnaParentPostStep(Real time, Int timeStepCount,
                                       Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  postStep();
}

void DecouplingLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) =
      -mRes2->intfVoltage()(0, 0) + mRes1->intfVoltage()(0, 0);
}

void DecouplingLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = -mRes1->intfCurrent()(0, 0) + mSrcCur1->get();
}

void DecouplingLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mStates);
  modifiedAttributes.push_back(mRightVector);
}

void DecouplingLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mStates);
}
