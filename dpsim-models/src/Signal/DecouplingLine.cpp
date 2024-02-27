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

DecouplingLine::DecouplingLine(String name, SimNode<Complex>::Ptr node1,
                               SimNode<Complex>::Ptr node2, Real resistance,
                               Real inductance, Real capacitance,
                               Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel), mResistance(resistance),
      mInductance(inductance), mCapacitance(capacitance), mNode1(node1),
      mNode2(node2), mStates(mAttributes->create<Matrix>("states")),
      mSrcCur1Ref(mAttributes->create<Complex>("i_src1")),
      mSrcCur2Ref(mAttributes->create<Complex>("i_src2")) {

  mSurgeImpedance = sqrt(inductance / capacitance);
  mDelay = sqrt(inductance * capacitance);
  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mRes1 = Resistor::make(name + "_r1", logLevel);
  mRes1->setParameters(mSurgeImpedance + resistance / 4);
  mRes1->connect({node1, SimNode<Complex>::GND});
  mRes2 = Resistor::make(name + "_r2", logLevel);
  mRes2->setParameters(mSurgeImpedance + resistance / 4);
  mRes2->connect({node2, SimNode<Complex>::GND});

  mSrc1 = CurrentSource::make(name + "_i1", logLevel);
  mSrc1->setParameters(0);
  mSrc1->connect({node1, SimNode<Complex>::GND});
  mSrcCur1 = mSrc1->mCurrentRef;
  mSrc2 = CurrentSource::make(name + "_i2", logLevel);
  mSrc2->setParameters(0);
  mSrc2->connect({node2, SimNode<Complex>::GND});
  mSrcCur2 = mSrc2->mCurrentRef;
}

DecouplingLine::DecouplingLine(String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcCur1Ref(mAttributes->create<Complex>("i_src1")),
      mSrcCur2Ref(mAttributes->create<Complex>("i_src2")) {

  mRes1 = Resistor::make(name + "_r1", logLevel);
  mRes2 = Resistor::make(name + "_r2", logLevel);
  mSrc1 = CurrentSource::make(name + "_i1", logLevel);
  mSrc2 = CurrentSource::make(name + "_i2", logLevel);

  mSrcCur1 = mSrc1->mCurrentRef;
  mSrcCur2 = mSrc2->mCurrentRef;
}

void DecouplingLine::setParameters(SimNode<Complex>::Ptr node1,
                                   SimNode<Complex>::Ptr node2, Real resistance,
                                   Real inductance, Real capacitance) {

  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;
  mNode1 = node1;
  mNode2 = node2;

  mSurgeImpedance = sqrt(inductance / capacitance);
  mDelay = sqrt(inductance * capacitance);
  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mRes1->setParameters(mSurgeImpedance + resistance / 4);
  mRes1->connect({node1, SimNode<Complex>::GND});
  mRes2->setParameters(mSurgeImpedance + resistance / 4);
  mRes2->connect({node2, SimNode<Complex>::GND});
  mSrc1->setParameters(0);
  mSrc1->connect({node1, SimNode<Complex>::GND});
  mSrc2->setParameters(0);
  mSrc2->connect({node2, SimNode<Complex>::GND});
}

void DecouplingLine::initialize(Real omega, Real timeStep) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  if (mNode1 == nullptr || mNode2 == nullptr)
    throw SystemError("nodes not initialized!");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  Complex volt1 = mNode1->initialSingleVoltage();
  Complex volt2 = mNode2->initialSingleVoltage();
  // TODO different initialization for lumped resistance?
  Complex initAdmittance = 1. / Complex(mResistance, omega * mInductance) +
                           Complex(0, omega * mCapacitance / 2);
  Complex cur1 = volt1 * initAdmittance -
                 volt2 / Complex(mResistance, omega * mInductance);
  Complex cur2 = volt2 * initAdmittance -
                 volt1 / Complex(mResistance, omega * mInductance);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

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

void DecouplingLine::PreStep::execute(Real time, Int timeStepCount) {
  mLine.step(time, timeStepCount);
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

void DecouplingLine::PostStep::execute(Real time, Int timeStepCount) {
  mLine.postStep();
}

Task::List DecouplingLine::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

IdentifiedObject::List DecouplingLine::getLineComponents() {
  return IdentifiedObject::List({mRes1, mRes2, mSrc1, mSrc2});
}
