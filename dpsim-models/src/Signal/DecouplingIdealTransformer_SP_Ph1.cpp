/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include "dpsim-models/SimNode.h"
#include "dpsim-models/TopologicalNode.h"
#include <cstdlib>
#include <dpsim-models/Signal/DecouplingIdealTransformer_SP_Ph1.h>

using namespace CPS;
using namespace CPS::SP::Ph1;
using namespace CPS::Signal;

DecouplingIdealTransformer_SP_Ph1::DecouplingIdealTransformer_SP_Ph1(
    String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSourceVoltageIntfVoltage(mAttributes->create<Complex>("v_intf")),
      mSourceVoltageIntfCurrent(mAttributes->create<Complex>("i_intf")),
      mSrcVoltageRef(mAttributes->create<Complex>("v_ref")),
      mSrcCurrentRef(mAttributes->create<Complex>("i_ref")) {

  mRes1 = Resistor::make(name + "_r1", logLevel);
  mRes2 = Resistor::make(name + "_r2", logLevel);
  mVoltageSrc = ControlledVoltageSource::make(name + "_v", logLevel);
  mCurrentSrc = ControlledCurrentSource::make(name + "_i", logLevel);

  mSrcVoltage = mVoltageSrc->mVoltageRef;
  mSrcCurrent = mCurrentSrc->mCurrentRef;
}

void DecouplingIdealTransformer_SP_Ph1::setParameters(
    SimNode<Complex>::Ptr node1, SimNode<Complex>::Ptr node2, Real delay,
    Matrix voltageSrcIntfCurr, Complex current1Extrap0, CouplingMethod method) {

  mNode1 = node1;
  mNode2 = node2;
  mVirtualNode = SimNode<Complex>::make(name() + "_virtual");

  mDelay = delay;
  mCouplingMethod = method;

  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    mExtrapolationDegree = 1;
  }

  mRes1->setParameters(mInternalSeriesResistance);
  mRes1->connect({node1, mVirtualNode});
  mRes2->setParameters(mInternalParallelResistance);
  mRes2->connect({node2, SimNode<Complex>::GND});
  mVoltageSrc->setParameters(Complex(0, 0));
  mVoltageSrcIntfCurr = voltageSrcIntfCurr;
  mCurrent1Extrap0 = current1Extrap0;
  mVoltageSrc->connect({SimNode<Complex>::GND, mVirtualNode});
  mCurrentSrc->setParameters(Complex(0, 0));
  mCurrentSrc->connect({SimNode<Complex>::GND, node2});
}

void DecouplingIdealTransformer_SP_Ph1::initialize(Real omega, Real timeStep) {
  if (mDelay <= 0) {
    mDelay = 0;
    mBufSize = 1;
    mAlpha = 1;
  } else {
    mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
    mAlpha = 1 - (mBufSize - mDelay / timeStep);
  }
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  mVoltageSrc->setIntfCurrent(mVoltageSrcIntfCurr);
  Complex cur1 = mVoltageSrc->mIntfCurrent->get()(0);
  Complex volt2 = mNode2->initialSingleVoltage() * RMS3PH_TO_PEAK1PH;

  mVirtualNode->setInitialVoltage(mNode1->initialSingleVoltage() *
                                      RMS3PH_TO_PEAK1PH -
                                  cur1 * mInternalSeriesResistance);

  SPDLOG_LOGGER_INFO(mSLog, "initial current: i_1 {}", cur1);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltage: v_2 {}", volt2);

  **mSrcVoltageRef = volt2;
  **mSrcCurrentRef = cur1;
  mVoltageSrc->setParameters(**mSrcVoltageRef);
  mCurrentSrc->setParameters(**mSrcCurrentRef);

  Matrix mSourceCurrentIntfVoltage(1, 1);
  mSourceCurrentIntfVoltage(0, 0) = std::abs(volt2);
  mCurrentSrc->setIntfVoltage(mSourceCurrentIntfVoltage);

  mVoltageSrc->setIntfVoltage(mSourceCurrentIntfVoltage);
  mVoltageSrc->setIntfCurrent(mVoltageSrcIntfCurr);

  **mSourceVoltageIntfVoltage = volt2;
  **mSourceVoltageIntfCurrent = mVoltageSrc->intfCurrent()(0, 0);

  // Resize ring buffers and initialize
  mCur1.resize(mBufSize, cur1);
  mVol2.resize(mBufSize, volt2);

  SPDLOG_LOGGER_INFO(mSLog, "Verify initial current: i_1 {}",
                     mCurrentSrc->intfCurrent()(0, 0));
  SPDLOG_LOGGER_INFO(mSLog, "Verify initial voltage: v_2 {}",
                     mVoltageSrc->intfVoltage()(0, 0));

  mCur1Extrap = std::vector<Complex>(mExtrapolationDegree + 1, Complex(0, 0));
  mCur1Extrap[0] = mCurrent1Extrap0;
  if (mExtrapolationDegree > 0) {
    mCur1Extrap[1] = mVoltageSrcIntfCurr(0, 0);
  }
  mVol2Extrap = std::vector<Complex>(mExtrapolationDegree + 1, volt2);
}

Complex
DecouplingIdealTransformer_SP_Ph1::interpolate(std::vector<Complex> &data) {
  Complex c1 = data[mBufIdx];
  Complex c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

Complex
DecouplingIdealTransformer_SP_Ph1::extrapolate(std::vector<Complex> &data) {
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

void DecouplingIdealTransformer_SP_Ph1::step(Real time, Int timeStepCount) {
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

void DecouplingIdealTransformer_SP_Ph1::PreStep::execute(Real time,
                                                         Int timeStepCount) {
  mITM.step(time, timeStepCount);
}

void DecouplingIdealTransformer_SP_Ph1::postStep() {
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

void DecouplingIdealTransformer_SP_Ph1::PostStep::execute(Real time,
                                                          Int timeStepCount) {
  mITM.postStep();
}

Task::List DecouplingIdealTransformer_SP_Ph1::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

IdentifiedObject::List DecouplingIdealTransformer_SP_Ph1::getComponents() {
  return IdentifiedObject::List({mRes1, mRes2, mVoltageSrc, mCurrentSrc});
}

TopologicalNode::Ptr DecouplingIdealTransformer_SP_Ph1::getVirtualNode() {
  return mVirtualNode;
}
