// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "dpsim-models/Attribute.h"
#include "dpsim-models/Definitions.h"
#include "dpsim-models/TopologicalNode.h"
#include <cstdlib>
#include <dpsim-models/DP/DP_Ph1_DecouplingIdealTransformer.h>

using namespace CPS;
using namespace CPS::DP::Ph1;

DP::Ph1::DecouplingIdealTransformer::DecouplingIdealTransformer(
    String uid, String name, Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSourceVoltageIntfVoltage(mAttributes->create<Complex>("v_src_intf")),
      mSourceVoltageIntfCurrent(mAttributes->create<Complex>("i_src_intf")),
      mSrcVoltageRef(mAttributes->create<Complex>("v_ref")),
      mSrcCurrentRef(mAttributes->create<Complex>("i_ref")) {

  setTerminalNumber(2);
  setVirtualNodeNumber(1);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);
}

void DP::Ph1::DecouplingIdealTransformer::setParameters(
    Real delay, Matrix voltageSrcIntfCurr, Complex current1Extrap0,
    CouplingMethod method) {

  mDelay = delay;
  mCouplingMethod = method;
  mVoltageSrcIntfCurr = voltageSrcIntfCurr;
  mCurrent1Extrap0 = current1Extrap0;

  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    mExtrapolationDegree = 1;
  }

  mParametersSet = true;
}

void DP::Ph1::DecouplingIdealTransformer::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  mRes1 = Resistor::make(**mName + "_r1", mLogLevel);
  mRes1->setParameters(mInternalSeriesResistance);
  mRes1->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  addMNASubComponent(mRes1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mRes2 = Resistor::make(**mName + "_r2", mLogLevel);
  mRes2->setParameters(mInternalParallelResistance);
  mRes2->connect({mTerminals[1]->node(), CPS::SimNode<Complex>::GND});
  addMNASubComponent(mRes2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mVoltageSrc = ControlledVoltageSource::make(**mName + "_v", mLogLevel);
  mVoltageSrc->setParameters(Complex(0, 0));
  mVoltageSrc->connect({CPS::SimNode<Complex>::GND, mVirtualNodes[0]});
  addMNASubComponent(mVoltageSrc, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mCurrentSrc = ControlledCurrentSource::make(**mName + "_i", mLogLevel);
  mCurrentSrc->setParameters(Complex(0, 0));
  mCurrentSrc->connect({CPS::SimNode<Complex>::GND, mTerminals[1]->node()});
  addMNASubComponent(mCurrentSrc, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrcVoltage = mVoltageSrc->mVoltageRef;
  mSrcCurrent = mCurrentSrc->mCurrentRef;
}

void DP::Ph1::DecouplingIdealTransformer::initializeParentFromNodesAndTerminals(
    Real frequency) {

  Complex cur1 = Complex(mVoltageSrcIntfCurr(0, 0), 0);
  Complex volt2 = initialSingleVoltage(1) * RMS3PH_TO_PEAK1PH;

  mVirtualNodes[0]->setInitialVoltage(initialSingleVoltage(0) *
                                          RMS3PH_TO_PEAK1PH -
                                      cur1 * mInternalSeriesResistance);

  SPDLOG_LOGGER_INFO(mSLog, "initial current: i_1 {}", cur1);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltage: v_2 {}", volt2);

  **mSrcVoltageRef = volt2;
  **mSrcCurrentRef = cur1;
  mVoltageSrc->setParameters(**mSrcVoltageRef);
  mCurrentSrc->setParameters(**mSrcCurrentRef);
}

void DP::Ph1::DecouplingIdealTransformer::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (mDelay <= 0) {
    mDelay = 0;
    mBufSize = 1;
    mAlpha = 1;
  } else {
    mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
    mAlpha = 1 - (mBufSize - mDelay / timeStep);
  }
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  Complex cur1 = **mSrcCurrentRef;
  Complex volt2 = **mSrcVoltageRef;
  Matrix mSourceCurrentIntfVoltage(1, 1);
  mSourceCurrentIntfVoltage(0, 0) = std::abs(volt2);
  mCurrentSrc->setIntfVoltage(mSourceCurrentIntfVoltage);

  mVoltageSrc->setIntfVoltage(mSourceCurrentIntfVoltage);
  mVoltageSrc->setIntfCurrent(mVoltageSrcIntfCurr);

  **mSourceVoltageIntfVoltage = volt2;
  **mSourceVoltageIntfCurrent = mVoltageSrc->intfCurrent()(0, 0);

  (**mIntfVoltage)(0, 0) = volt2;
  (**mIntfCurrent)(0, 0) = cur1;

  // Resize ring buffers and initialize
  mCur1.resize(mBufSize, cur1);
  mVol2.resize(mBufSize, volt2);

  SPDLOG_LOGGER_INFO(mSLog, "Verify initial current: i_1 {}",
                     mCurrentSrc->intfCurrent()(0, 0));
  SPDLOG_LOGGER_INFO(mSLog, "Verify initial voltage: v_2 {}",
                     mVoltageSrc->intfVoltage()(0, 0));

  mCur1Extrap = std::vector<Complex>(mExtrapolationDegree + 1);
  mCur1Extrap[0] = mCurrent1Extrap0;
  if (mExtrapolationDegree > 0) {
    mCur1Extrap[1] = mVoltageSrcIntfCurr(0, 0);
  }
  mVol2Extrap = std::vector<Complex>(mExtrapolationDegree + 1);
}

Complex
DP::Ph1::DecouplingIdealTransformer::interpolate(std::vector<Complex> &data) {
  Complex c1 = data[mBufIdx];
  Complex c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

Complex
DP::Ph1::DecouplingIdealTransformer::extrapolate(std::vector<Complex> &data) {
  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    Complex c1 = data[mMacroBufIdx];
    Complex c2 =
        mMacroBufIdx == mExtrapolationDegree ? data[0] : data[mMacroBufIdx + 1];
    Real delayFraction =
        (mDelay * (mBufIdx + 1)) / static_cast<float>(mBufSize);
    Real tEval = mDelay + delayFraction;
    return ((c2 - c1) / mDelay) * tEval + c1;
  } else {
    return data[mMacroBufIdx];
  }
}

void DP::Ph1::DecouplingIdealTransformer::step(Real time, Int timeStepCount) {
  Complex volt1, cur2;
  if (mCouplingMethod == CouplingMethod::DELAY) {
    volt1 = interpolate(mVol2);
    cur2 = interpolate(mCur1);
  } else {
    volt1 = extrapolate(mVol2Extrap);
    cur2 = extrapolate(mCur1Extrap);
  }

  // Update voltage and current
  **mSrcVoltageRef = volt1;
  **mSrcCurrentRef = cur2;
  **mSourceVoltageIntfVoltage = mVoltageSrc->intfVoltage()(0, 0);
  **mSourceVoltageIntfCurrent = mVoltageSrc->intfCurrent()(0, 0);

  mSrcVoltage->set(**mSrcVoltageRef);
  mSrcCurrent->set(**mSrcCurrentRef);
}

void DP::Ph1::DecouplingIdealTransformer::postStep() {
  // Update ringbuffers with new values
  mCur1[mBufIdx] = mVoltageSrc->intfCurrent()(0, 0);
  mVol2[mBufIdx] = -mCurrentSrc->intfVoltage()(0, 0);

  mBufIdx++;
  if (mBufIdx == mBufSize) {
    mCur1Extrap[mMacroBufIdx] = mCur1[mBufIdx - 1];
    mVol2Extrap[mMacroBufIdx] = mVol2[mBufIdx - 1];
    mMacroBufIdx++;
    if (mMacroBufIdx == mExtrapolationDegree + 1) {
      mMacroBufIdx = 0;
    }
    mBufIdx = 0;
  }
}

void DP::Ph1::DecouplingIdealTransformer::mnaParentPreStep(Real time,
                                                           Int timeStepCount) {
  step(time, timeStepCount);
  mVoltageSrc->mnaPreStep(time, timeStepCount);
  mCurrentSrc->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::DecouplingIdealTransformer::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  postStep();
}

void DP::Ph1::DecouplingIdealTransformer::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) = mVoltageSrc->intfVoltage()(0, 0);
}

void DP::Ph1::DecouplingIdealTransformer::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mVoltageSrc->intfCurrent()(0, 0);
}

void DP::Ph1::DecouplingIdealTransformer::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mStates);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::DecouplingIdealTransformer::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mStates);
}
