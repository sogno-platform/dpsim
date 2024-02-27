/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/DecouplingLineEMT.h>

using namespace CPS;
using namespace CPS::EMT::Ph1;
using namespace CPS::Signal;

DecouplingLineEMT::DecouplingLineEMT(String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcCur1Ref(mAttributes->create<Real>("i_src1")),
      mSrcCur2Ref(mAttributes->create<Real>("i_src2")) {

  mRes1 = Resistor::make(name + "_r1", logLevel);
  mRes2 = Resistor::make(name + "_r2", logLevel);
  mSrc1 = CurrentSource::make(name + "_i1", logLevel);
  mSrc2 = CurrentSource::make(name + "_i2", logLevel);

  mSrcCur1 = mSrc1->mCurrentRef;
  mSrcCur2 = mSrc2->mCurrentRef;
}

void DecouplingLineEMT::setParameters(SimNode<Real>::Ptr node1,
                                      SimNode<Real>::Ptr node2, Real resistance,
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

  mRes1->setParameters(mSurgeImpedance + mResistance / 4);
  mRes1->connect({node1, SimNode<Real>::GND});
  mRes2->setParameters(mSurgeImpedance + mResistance / 4);
  mRes2->connect({node2, SimNode<Real>::GND});
  mSrc1->setParameters(0);
  mSrc1->connect({node1, SimNode<Real>::GND});
  mSrc2->setParameters(0);
  mSrc2->connect({node2, SimNode<Real>::GND});
}

void DecouplingLineEMT::initialize(Real omega, Real timeStep) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // Initialization based on static PI-line model
  Complex volt1 = mNode1->initialSingleVoltage();
  Complex volt2 = mNode2->initialSingleVoltage();
  Complex initAdmittance = 1. / Complex(mResistance, omega * mInductance) +
                           Complex(0, omega * mCapacitance / 2);
  Complex cur1 = volt1 * initAdmittance -
                 volt2 / Complex(mResistance, omega * mInductance);
  Complex cur2 = volt2 * initAdmittance -
                 volt1 / Complex(mResistance, omega * mInductance);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  // Resize ring buffers and initialize
  mVolt1.resize(mBufSize, volt1.real());
  mVolt2.resize(mBufSize, volt2.real());
  mCur1.resize(mBufSize, cur1.real());
  mCur2.resize(mBufSize, cur2.real());
}

Real DecouplingLineEMT::interpolate(std::vector<Real> &data) {
  // linear interpolation of the nearest values
  Real c1 = data[mBufIdx];
  Real c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

void DecouplingLineEMT::step(Real time, Int timeStepCount) {
  Real volt1 = interpolate(mVolt1);
  Real volt2 = interpolate(mVolt2);
  Real cur1 = interpolate(mCur1);
  Real cur2 = interpolate(mCur2);
  Real denom =
      (mSurgeImpedance + mResistance / 4) * (mSurgeImpedance + mResistance / 4);

  if (timeStepCount == 0) {
    // initialization
    **mSrcCur1Ref = cur1 - volt1 / (mSurgeImpedance + mResistance / 4);
    **mSrcCur2Ref = cur2 - volt2 / (mSurgeImpedance + mResistance / 4);
  } else {
    // Update currents
    **mSrcCur1Ref = -mSurgeImpedance / denom *
                        (volt2 + (mSurgeImpedance - mResistance / 4) * cur2) -
                    mResistance / 4 / denom *
                        (volt1 + (mSurgeImpedance - mResistance / 4) * cur1);
    **mSrcCur2Ref = -mSurgeImpedance / denom *
                        (volt1 + (mSurgeImpedance - mResistance / 4) * cur1) -
                    mResistance / 4 / denom *
                        (volt2 + (mSurgeImpedance - mResistance / 4) * cur2);
  }
  mSrcCur1->set(**mSrcCur1Ref);
  mSrcCur2->set(**mSrcCur2Ref);
}

void DecouplingLineEMT::PreStep::execute(Real time, Int timeStepCount) {
  mLine.step(time, timeStepCount);
}

void DecouplingLineEMT::postStep() {
  // Update ringbuffers with new values
  mVolt1[mBufIdx] = -mRes1->intfVoltage()(0, 0);
  mVolt2[mBufIdx] = -mRes2->intfVoltage()(0, 0);
  mCur1[mBufIdx] = -mRes1->intfCurrent()(0, 0) + mSrcCur1->get().real();
  mCur2[mBufIdx] = -mRes2->intfCurrent()(0, 0) + mSrcCur2->get().real();

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;
}

void DecouplingLineEMT::PostStep::execute(Real time, Int timeStepCount) {
  mLine.postStep();
}

Task::List DecouplingLineEMT::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

IdentifiedObject::List DecouplingLineEMT::getLineComponents() {
  return IdentifiedObject::List({mRes1, mRes2, mSrc1, mSrc2});
}
