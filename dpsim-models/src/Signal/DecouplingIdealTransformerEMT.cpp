/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
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
                                      Real delay) {

  mNode1 = node1;
  mNode2 = node2;

  mDelay = delay;
  mVoltageSrc->setParameters(0);
  mVoltageSrc->connect({node1, SimNode<Real>::GND});
  mCurrentSrc->setParameters(0);
  mCurrentSrc->connect({node2, SimNode<Real>::GND});
}

void DecouplingIdealTransformerEMT::initialize(Real omega, Real timeStep) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);


  Complex volt1 = mNode1->initialSingleVoltage();
  Complex volt2 = mNode2->initialSingleVoltage();
  Complex cur1 = mVoltageSrc->mIntfCurrent->get()(0);
  Complex cur2 = mCurrentSrc->mIntfCurrent->get()(0);

  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  // Resize ring buffers and initialize
  mVolt1.resize(mBufSize, volt1.real());
  mVolt2.resize(mBufSize, volt2.real());
  mCur1.resize(mBufSize, cur1.real());
  mCur2.resize(mBufSize, cur2.real());
}

Real DecouplingIdealTransformerEMT::interpolate(std::vector<Real> &data) {
  // linear interpolation of the nearest values
  Real c1 = data[mBufIdx];
  Real c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

void DecouplingIdealTransformerEMT::step(Real time, Int timeStepCount) {
  Real volt1 = interpolate(mVolt1);
  Real volt2 = interpolate(mVolt2);
  Real cur1 = interpolate(mCur1);
  Real cur2 = interpolate(mCur2);

  if (timeStepCount == 0) {
    // initialization
    **mSrcVoltageRef = volt1;
    **mSrcCurrentRef = cur2;
  } else {
    // Update currents
    **mSrcVoltageRef = volt1;
    **mSrcCurrentRef = cur2;
  }
  mSrcVoltage->set(**mSrcVoltageRef);
  mSrcCurrent->set(**mSrcCurrentRef);
}

void DecouplingIdealTransformerEMT::PreStep::execute(Real time, Int timeStepCount) {
  mITM.step(time, timeStepCount);
}

void DecouplingIdealTransformerEMT::postStep() {
  // Update ringbuffers with new values
  mVolt1[mBufIdx] = mVoltageSrc->intfVoltage()(0, 0);
  mVolt2[mBufIdx] = mCurrentSrc->intfVoltage()(0, 0);
  mCur1[mBufIdx] = mVoltageSrc->intfCurrent()(0, 0);
  mCur2[mBufIdx] = mCurrentSrc->intfCurrent()(0, 0);

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;
}

void DecouplingIdealTransformerEMT::PostStep::execute(Real time, Int timeStepCount) {
  mITM.postStep();
}

Task::List DecouplingIdealTransformerEMT::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}
