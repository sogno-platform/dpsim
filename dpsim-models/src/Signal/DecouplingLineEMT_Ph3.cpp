/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/Definitions.h"
#include "dpsim-models/EMT/EMT_Ph3_Resistor.h"
#include "dpsim-models/MathUtils.h"
#include <Eigen/src/Core/Array.h>
#include <dpsim-models/Signal/DecouplingLineEMT_Ph3.h>

using namespace CPS;
using namespace CPS::EMT::Ph3;
using namespace CPS::Signal;

DecouplingLineEMT_Ph3::DecouplingLineEMT_Ph3(String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcCur1Ref(mAttributes->create<Matrix>("i_src1", Matrix::Zero(3, 1))),
      mSrcCur2Ref(mAttributes->create<Matrix>("i_src2", Matrix::Zero(3, 1))) {

  mRes1 = EMT::Ph3::Resistor::make(name + "_r1", logLevel);
  mRes2 = EMT::Ph3::Resistor::make(name + "_r2", logLevel);
  mSrc1 = ControlledCurrentSource::make(name + "_i1", logLevel);
  mSrc2 = ControlledCurrentSource::make(name + "_i2", logLevel);
}

void DecouplingLineEMT_Ph3::setParameters(SimNode<Real>::Ptr node1,
                                      SimNode<Real>::Ptr node2, Matrix resistance,
                                      Matrix inductance, Matrix capacitance) {

  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;
  mNode1 = node1;
  mNode2 = node2;

  mSurgeImpedance = (inductance * capacitance.inverse()).array().sqrt();
  mDelay = (inductance.array() * capacitance.array()).sqrt().maxCoeff();
  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mRes1->setParameters(Math::singlePhaseParameterToThreePhase(mSurgeImpedance(0,0) + mResistance(0,0) / 4));
  mRes1->connect({SimNode<Real>::GND, node1});
  mRes2->setParameters(Math::singlePhaseParameterToThreePhase(mSurgeImpedance(0,0) + mResistance(0,0) / 4));
  /*Notice that, as opposed to the DecouplingLine Ph1, this resistor is connected from GND to node2,
   since currently the Ph3 resistor has the opposite sign convention for voltage and current, compared to the Ph1 countepart.*/
  mRes2->connect({SimNode<Real>::GND, node2});
  mSrc1->setParameters(Matrix::Zero(3,1));
  mSrc1->connect({node1, SimNode<Real>::GND});
  mSrc2->setParameters(Matrix::Zero(3,1));
  mSrc2->connect({node2, SimNode<Real>::GND});

  mSrcCur1 = mSrc1->mCurrentRef;
  mSrcCur2 = mSrc2->mCurrentRef;
}

void DecouplingLineEMT_Ph3::initialize(Real omega, Real timeStep) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // Initialization based on static PI-line model
  MatrixComp volt1 = -mNode1->initialVoltage();
  MatrixComp volt2 = -mNode2->initialVoltage();

  MatrixComp initAdmittance = (mResistance +  Complex(0, omega) * mInductance).inverse() +
                              Complex(0, omega) * mCapacitance / 2;
  MatrixComp cur1 = initAdmittance * volt1 -
                    (mResistance +  Complex(0, omega) * mInductance).inverse() * volt2;
  MatrixComp cur2 = initAdmittance * volt2 -
                    (mResistance +  Complex(0, omega) * mInductance).inverse() * volt1;

  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  // Resize ring buffers and initialize
  mVolt1 = volt1.real().transpose().replicate(mBufSize, 1);
  mVolt2 = volt2.real().transpose().replicate(mBufSize, 1);
  mCur1 = cur1.real().transpose().replicate(mBufSize, 1);
  mCur2 = cur2.real().transpose().replicate(mBufSize, 1);
}

Matrix DecouplingLineEMT_Ph3::interpolate(Matrix &data) {
  // linear interpolation of the nearest values
  Matrix c1 = data.row(mBufIdx);
  Matrix c2 = mBufIdx == mBufSize - 1 ? data.row(0) : data.row(mBufIdx + 1);
  return (mAlpha * c1 + (1 - mAlpha) * c2).transpose();
}

void DecouplingLineEMT_Ph3::step(Real time, Int timeStepCount) {
  Matrix volt1 = interpolate(mVolt1);
  Matrix volt2 = interpolate(mVolt2);
  Matrix cur1 = interpolate(mCur1);
  Matrix cur2 = interpolate(mCur2);
  Matrix denom = (mSurgeImpedance + (mResistance / 4)) * (mSurgeImpedance + (mResistance / 4));

  if (timeStepCount == 0) {
    // initialization
    **mSrcCur1Ref = cur1 - (mSurgeImpedance + mResistance / 4).inverse() * volt1;
    **mSrcCur2Ref = cur2 - (mSurgeImpedance + mResistance / 4).inverse() * volt2;
  } else {
    // Update currents
    **mSrcCur1Ref = -mSurgeImpedance * denom.inverse() *
                        (volt2 + (mSurgeImpedance - mResistance / 4) * cur2) -
                    mResistance / 4 * denom.inverse() *
                        (volt1 + (mSurgeImpedance - mResistance / 4) * cur1);
    **mSrcCur2Ref = -mSurgeImpedance * denom.inverse() *
                        (volt1 + (mSurgeImpedance - mResistance / 4) * cur1) -
                    mResistance / 4 * denom.inverse() *
                        (volt2 + (mSurgeImpedance - mResistance / 4) * cur2);
  }
  mSrcCur1->set(**mSrcCur1Ref);
  mSrcCur2->set(**mSrcCur2Ref);
}

void DecouplingLineEMT_Ph3::PreStep::execute(Real time, Int timeStepCount) {
  mLine.step(time, timeStepCount);
}

void DecouplingLineEMT_Ph3::postStep() {
  // Update ringbuffers with new values  
  mVolt1.row(mBufIdx) = -mRes1->intfVoltage().transpose();
  mVolt2.row(mBufIdx) = -mRes2->intfVoltage().transpose();
  mCur1.row(mBufIdx) = -mRes1->intfCurrent().transpose() + mSrcCur1->get().real().transpose();
  mCur2.row(mBufIdx) = -mRes2->intfCurrent().transpose() + mSrcCur2->get().real().transpose();

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;
}

void DecouplingLineEMT_Ph3::PostStep::execute(Real time, Int timeStepCount) {
  mLine.postStep();
}

Task::List DecouplingLineEMT_Ph3::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<PostStep>(*this)});
}

IdentifiedObject::List DecouplingLineEMT_Ph3::getLineComponents() {
  return IdentifiedObject::List({mRes1, mRes2, mSrc1, mSrc2});
}
