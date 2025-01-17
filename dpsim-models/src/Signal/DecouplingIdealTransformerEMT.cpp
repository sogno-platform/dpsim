/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include <algorithm>
#include <dpsim-models/Signal/DecouplingIdealTransformerEMT.h>
#include <iostream>
#include <string>

using namespace CPS;
using namespace CPS::EMT::Ph1;
using namespace CPS::Signal;

DecouplingIdealTransformerEMT::DecouplingIdealTransformerEMT(String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcVoltageRef(mAttributes->create<Real>("v_src")),
      mSrcCurrentRef(mAttributes->create<Real>("i_src")) {

  mVoltageSrc = VoltageSource::make(name + "_v", logLevel);
  mCurrentSrc = CurrentSource::make(name + "_i", logLevel);

  mSrcVoltage = mVoltageSrc->mVoltageRef;
  mSrcCurrent = mCurrentSrc->mCurrentRef;
}

void DecouplingIdealTransformerEMT::setParameters(SimNode<Real>::Ptr node1,
                                      SimNode<Real>::Ptr node2,
                                      Real delay, Eigen::MatrixXd voltageSrcIntfCurr, CouplingMethod method) {

  mNode1 = node1;
  mNode2 = node2;

  mDelay = delay;
  mCouplingMethod = method;

  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    mExtrapolationDegree = 1;
  }

  mVoltageSrc->setParameters(0);
  mVoltageSrcIntfCurr = voltageSrcIntfCurr;
  mVoltageSrc->connect({SimNode<Real>::GND, node1});
  mCurrentSrc->setParameters(0);
  mCurrentSrc->connect({SimNode<Real>::GND, node2});
}

void DecouplingIdealTransformerEMT::initialize(Real omega, Real timeStep) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  // mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  mVoltageSrc->setIntfCurrent(mVoltageSrcIntfCurr);
  Complex cur1 = mVoltageSrc->mIntfCurrent->get()(0);
  Complex volt2 = mNode2->initialSingleVoltage() * RMS3PH_TO_PEAK1PH;

  SPDLOG_LOGGER_INFO(mSLog, "initial current: i_1 {}", cur1);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltage: v_2 {}", volt2);

  // Resize ring buffers and initialize
  mCur1.resize(mBufSize, cur1.real()); // TODO: add initial value to setParameters?
  mVol2.resize(mBufSize, volt2.real());

  mCur1Extrap.resize(mExtrapolationDegree + 1, cur1.real());
  mVol2Extrap.resize(mExtrapolationDegree + 1, volt2.real());
}

Real DecouplingIdealTransformerEMT::interpolate(std::vector<Real> &data) {
  Real c1 = data[mBufIdx];
  Real c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

Real DecouplingIdealTransformerEMT::extrapolate(std::vector<Real> &data) {
  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    Real c1 = data[mMacroBufIdx];
    Real c2 = mMacroBufIdx == mExtrapolationDegree ? data[0] : data[mMacroBufIdx + 1];
    Real delayFraction = (mDelay*(mBufIdx+1)) / static_cast<float>(mBufSize);
    Real tEval = mDelay + delayFraction;
    return ((c2 - c1)/mDelay) * tEval + c1;
  } else {
    return data[mMacroBufIdx];
  }
}

void DecouplingIdealTransformerEMT::step(Real time, Int timeStepCount) {
  Real volt1, cur2;
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
  mSrcVoltage->set(**mSrcVoltageRef);
  mSrcCurrent->set(**mSrcCurrentRef);
}

void DecouplingIdealTransformerEMT::PreStep::execute(Real time, Int timeStepCount) {
  mITM.step(time, timeStepCount);
}

void DecouplingIdealTransformerEMT::postStep() {
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

void DecouplingIdealTransformerEMT::PostStep::execute(Real time, Int timeStepCount) {
  mITM.postStep();
}

Task::List DecouplingIdealTransformerEMT::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

IdentifiedObject::List DecouplingIdealTransformerEMT::getComponents() {
  return IdentifiedObject::List({mVoltageSrc, mCurrentSrc});
}
