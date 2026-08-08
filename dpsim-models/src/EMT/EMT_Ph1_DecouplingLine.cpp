/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_DecouplingLine.h>

using namespace CPS;
using namespace CPS::EMT::Ph1;

EMT::Ph1::DecouplingLine::DecouplingLine(String uid, String name,
                                         Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcCur1Ref(mAttributes->create<Real>("i_src1")),
      mSrcCur2Ref(mAttributes->create<Real>("i_src2")) {

  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
}

void EMT::Ph1::DecouplingLine::setParameters(Real resistance, Real inductance,
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

void EMT::Ph1::DecouplingLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  mRes1 = Resistor::make(**mName + "_r1", mLogLevel);
  mRes1->setParameters(mSurgeImpedance + mResistance / 4);
  mRes1->connect({mTerminals[0]->node(), EMT::SimNode::GND});
  addMNASubComponent(mRes1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mRes2 = Resistor::make(**mName + "_r2", mLogLevel);
  mRes2->setParameters(mSurgeImpedance + mResistance / 4);
  mRes2->connect({mTerminals[1]->node(), EMT::SimNode::GND});
  addMNASubComponent(mRes2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSrc1 = CurrentSource::make(**mName + "_i1", mLogLevel);
  mSrc1->setParameters(0);
  mSrc1->connect({mTerminals[0]->node(), EMT::SimNode::GND});
  addMNASubComponent(mSrc1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrc2 = CurrentSource::make(**mName + "_i2", mLogLevel);
  mSrc2->setParameters(0);
  mSrc2->connect({mTerminals[1]->node(), EMT::SimNode::GND});
  addMNASubComponent(mSrc2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrcCur1 = mSrc1->mCurrentRef;
  mSrcCur2 = mSrc2->mCurrentRef;
}

void EMT::Ph1::DecouplingLine::initializeParentFromNodesAndTerminals(
    Real frequency) {
  (**mIntfVoltage)(0, 0) =
      (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
}

void EMT::Ph1::DecouplingLine::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // Initialization based on static PI-line model
  Complex volt1 = initialSingleVoltage(0);
  Complex volt2 = initialSingleVoltage(1);
  Complex initAdmittance = 1. / Complex(mResistance, omega * mInductance) +
                           Complex(0, omega * mCapacitance / 2);
  Complex cur1 = volt1 * initAdmittance -
                 volt2 / Complex(mResistance, omega * mInductance);
  Complex cur2 = volt2 * initAdmittance -
                 volt1 / Complex(mResistance, omega * mInductance);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  (**mIntfCurrent)(0, 0) = cur1.real();

  // Resize ring buffers and initialize
  mVolt1.resize(mBufSize, volt1.real());
  mVolt2.resize(mBufSize, volt2.real());
  mCur1.resize(mBufSize, cur1.real());
  mCur2.resize(mBufSize, cur2.real());
}

Real EMT::Ph1::DecouplingLine::interpolate(std::vector<Real> &data) {
  // linear interpolation of the nearest values
  Real c1 = data[mBufIdx];
  Real c2 = mBufIdx == mBufSize - 1 ? data[0] : data[mBufIdx + 1];
  return mAlpha * c1 + (1 - mAlpha) * c2;
}

void EMT::Ph1::DecouplingLine::step(Real time, Int timeStepCount) {
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

void EMT::Ph1::DecouplingLine::postStep() {
  // Update ringbuffers with new values
  mVolt1[mBufIdx] = -mRes1->intfVoltage()(0, 0);
  mVolt2[mBufIdx] = -mRes2->intfVoltage()(0, 0);
  mCur1[mBufIdx] = -mRes1->intfCurrent()(0, 0) + mSrcCur1->get().real();
  mCur2[mBufIdx] = -mRes2->intfCurrent()(0, 0) + mSrcCur2->get().real();

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;
}

void EMT::Ph1::DecouplingLine::mnaParentPreStep(Real time, Int timeStepCount) {
  step(time, timeStepCount);
  mSrc1->mnaPreStep(time, timeStepCount);
  mSrc2->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::DecouplingLine::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  postStep();
}

void EMT::Ph1::DecouplingLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) =
      -mRes2->intfVoltage()(0, 0) + mRes1->intfVoltage()(0, 0);
}

void EMT::Ph1::DecouplingLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = -mRes1->intfCurrent()(0, 0) + mSrcCur1->get().real();
}

void EMT::Ph1::DecouplingLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mStates);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph1::DecouplingLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mStates);
}
