/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include <dpsim-models/Signal/DecouplingIdealTransformer_EMT_Ph3.h>

using namespace CPS;
using namespace CPS::EMT::Ph3;
using namespace CPS::Signal;

DecouplingIdealTransformer_EMT_Ph3::DecouplingIdealTransformer_EMT_Ph3(String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSourceVoltageIntfVoltage(mAttributes->create<Matrix>("v_intf", Matrix::Zero(3, 1))),
      mSourceVoltageIntfCurrent(mAttributes->create<Matrix>("i_intf", Matrix::Zero(3, 1))),
      mSrcVoltageRef(mAttributes->create<Matrix>("v_ref", Matrix::Zero(3, 1))),
      mSrcCurrentRef(mAttributes->create<Matrix>("i_ref", Matrix::Zero(3, 1))) {

  mVoltageSrc = ControlledVoltageSource::make(name + "_v", logLevel);
  mCurrentSrc = ControlledCurrentSource::make(name + "_i", logLevel);

  mSrcVoltage = mVoltageSrc->mVoltageRef;
  mSrcCurrent = mCurrentSrc->mCurrentRef;
}

void DecouplingIdealTransformer_EMT_Ph3::setParameters(SimNode<Real>::Ptr node1,
                                      SimNode<Real>::Ptr node2,
                                      Real delay, Matrix voltageSrcIntfCurr,
                                      Matrix current1Extrap0, CouplingMethod method) {

  mNode1 = node1;
  mNode2 = node2;

  mDelay = delay;
  mCouplingMethod = method;

  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    mExtrapolationDegree = 1;
  }

  mVoltageSrc->setParameters(Matrix::Zero(3,1));
  mVoltageSrcIntfCurr = voltageSrcIntfCurr;
  mCurrent1Extrap0 = current1Extrap0;
  mVoltageSrc->connect({SimNode<Real>::GND, node1});
  mCurrentSrc->setParameters(Matrix::Zero(3,1));
  mCurrentSrc->connect({node2, SimNode<Real>::GND});
}

void DecouplingIdealTransformer_EMT_Ph3::initialize(Real omega, Real timeStep) {
  if (mDelay <= 0) {
    mDelay = 0;
    mBufSize = 1;
    mAlpha = 1;
  }
  else {
    mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
    mAlpha = 1 - (mBufSize - mDelay / timeStep);
  }
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  mVoltageSrc->setIntfCurrent(mVoltageSrcIntfCurr);
  MatrixComp cur1 = mVoltageSrc->mIntfCurrent->get();
  MatrixComp volt2 = mNode2->initialVoltage();

  SPDLOG_LOGGER_INFO(mSLog, "initial current: i_1 {}", cur1);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltage: v_2 {}", volt2);

  **mSrcVoltageRef = volt2.real();
  **mSrcCurrentRef = cur1.real();
  mVoltageSrc->setParameters(**mSrcVoltageRef);
  mCurrentSrc->setParameters(**mSrcCurrentRef);

  Matrix mSourceCurrentIntfVoltage = volt2.real();
  mCurrentSrc->setIntfVoltage(mSourceCurrentIntfVoltage);

  mVoltageSrc->setIntfVoltage(mSourceCurrentIntfVoltage);
  mVoltageSrc->setIntfCurrent(mVoltageSrcIntfCurr);

  **mSourceVoltageIntfVoltage = volt2.real();
  **mSourceVoltageIntfCurrent = mVoltageSrc->intfCurrent();

  // Resize ring buffers and initialize
  mCur1 = cur1.real().transpose().replicate(mBufSize, 1);
  mVol2 = volt2.real().transpose().replicate(mBufSize, 1);

  SPDLOG_LOGGER_INFO(mSLog, "Verify initial current: i_1 {}", mCurrentSrc->intfCurrent()(0, 0));
  SPDLOG_LOGGER_INFO(mSLog, "Verify initial voltage: v_2 {}", mVoltageSrc->intfVoltage()(0, 0));

  mCur1Extrap = Matrix(mExtrapolationDegree + 1, 3);
  mCur1Extrap.row(0) = mCurrent1Extrap0.real().transpose();
  if (mExtrapolationDegree > 0) {
    mCur1Extrap.row(1) = mVoltageSrcIntfCurr.transpose();
  }
  mVol2Extrap = volt2.real().transpose().replicate(mExtrapolationDegree + 1, 1);
}

Matrix DecouplingIdealTransformer_EMT_Ph3::interpolate(Matrix &data) {
  Matrix c1 = data.row(mBufIdx);
  Matrix c2 = mBufIdx == mBufSize - 1 ? data.row(0) : data.row(mBufIdx + 1);
  return (mAlpha * c1 + (1 - mAlpha) * c2).transpose();
}

Matrix DecouplingIdealTransformer_EMT_Ph3::extrapolate(Matrix &data) {
  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    Matrix c1 = data.row(mMacroBufIdx);
    Matrix c2 = mMacroBufIdx == mExtrapolationDegree ? data.row(0) : data.row(mMacroBufIdx + 1);
    Real delayFraction = (mDelay*(mBufIdx+1)) / static_cast<float>(mBufSize);
    Real tEval = mDelay + delayFraction;
    return (((c2 - c1)/mDelay) * tEval + c1).transpose();
  } else {
    return (data.row(mMacroBufIdx)).transpose();
  }
}

void DecouplingIdealTransformer_EMT_Ph3::step(Real time, Int timeStepCount) {
  Matrix volt1, cur2;
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
  **mSourceVoltageIntfVoltage = mVoltageSrc->intfVoltage();
  **mSourceVoltageIntfCurrent = mVoltageSrc->intfCurrent();

  mSrcVoltage->set(**mSrcVoltageRef);
  mSrcCurrent->set(**mSrcCurrentRef);
}

void DecouplingIdealTransformer_EMT_Ph3::PreStep::execute(Real time, Int timeStepCount) {
  mITM.step(time, timeStepCount);
}

void DecouplingIdealTransformer_EMT_Ph3::postStep() {
  // Update ringbuffers with new values
  mCur1.row(mBufIdx) = mVoltageSrc->intfCurrent().transpose();
  mVol2.row(mBufIdx) = -mCurrentSrc->intfVoltage().transpose();

  mBufIdx++;
  if (mBufIdx == mBufSize) {
    mCur1Extrap.row(mMacroBufIdx) = mCur1.row(mBufIdx - 1);
    mVol2Extrap.row(mMacroBufIdx) = mVol2.row(mBufIdx - 1);
    mMacroBufIdx++;
    if (mMacroBufIdx == mExtrapolationDegree + 1) {
      mMacroBufIdx = 0;
    }
    mBufIdx = 0;
  }
}

void DecouplingIdealTransformer_EMT_Ph3::PostStep::execute(Real time, Int timeStepCount) {
  mITM.postStep();
}

Task::List DecouplingIdealTransformer_EMT_Ph3::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

IdentifiedObject::List DecouplingIdealTransformer_EMT_Ph3::getComponents() {
  return IdentifiedObject::List({mVoltageSrc, mCurrentSrc});
}
