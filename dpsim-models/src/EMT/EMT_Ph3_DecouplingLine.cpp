/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include "dpsim-models/MathUtils.h"
#include <Eigen/src/Core/Array.h>
#include <dpsim-models/EMT/EMT_Ph3_DecouplingLine.h>

using namespace CPS;
using namespace CPS::EMT::Ph3;

EMT::Ph3::DecouplingLine::DecouplingLine(String uid, String name,
                                         Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSrcCur1Ref(mAttributes->create<Matrix>("i_src1", Matrix::Zero(3, 1))),
      mSrcCur2Ref(mAttributes->create<Matrix>("i_src2", Matrix::Zero(3, 1))) {

  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::DecouplingLine::setParameters(Matrix resistance,
                                             Matrix inductance,
                                             Matrix capacitance) {

  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;

  mSurgeImpedance = (inductance * capacitance.inverse()).array().sqrt();
  mDelay = (inductance.array() * capacitance.array()).sqrt().maxCoeff();
  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mParametersSet = true;
}

void EMT::Ph3::DecouplingLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  mRes1 = EMT::Ph3::Resistor::make(**mName + "_r1", mLogLevel);
  mRes1->setParameters(Math::singlePhaseParameterToThreePhase(
      mSurgeImpedance(0, 0) + mResistance(0, 0) / 4));
  mRes1->connect({EMT::SimNode::GND, mTerminals[0]->node()});
  addMNASubComponent(mRes1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mRes2 = EMT::Ph3::Resistor::make(**mName + "_r2", mLogLevel);
  mRes2->setParameters(Math::singlePhaseParameterToThreePhase(
      mSurgeImpedance(0, 0) + mResistance(0, 0) / 4));
  /*Notice that, as opposed to the DecouplingLine Ph1, this resistor is connected from GND to node2,
   since currently the Ph3 resistor has the opposite sign convention for voltage and current, compared to the Ph1 countepart.*/
  mRes2->connect({EMT::SimNode::GND, mTerminals[1]->node()});
  addMNASubComponent(mRes2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSrc1 = ControlledCurrentSource::make(**mName + "_i1", mLogLevel);
  mSrc1->setParameters(Matrix::Zero(3, 1));
  mSrc1->connect({mTerminals[0]->node(), EMT::SimNode::GND});
  addMNASubComponent(mSrc1, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrc2 = ControlledCurrentSource::make(**mName + "_i2", mLogLevel);
  mSrc2->setParameters(Matrix::Zero(3, 1));
  mSrc2->connect({mTerminals[1]->node(), EMT::SimNode::GND});
  addMNASubComponent(mSrc2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrcCur1 = mSrc1->mCurrentRef;
  mSrcCur2 = mSrc2->mCurrentRef;
}

void EMT::Ph3::DecouplingLine::initializeParentFromNodesAndTerminals(
    Real frequency) {
  **mIntfVoltage = (initialVoltage(1) - initialVoltage(0)).real();
}

void EMT::Ph3::DecouplingLine::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // Initialization based on static PI-line model
  MatrixComp volt1 = -initialVoltage(0);
  MatrixComp volt2 = -initialVoltage(1);

  MatrixComp initAdmittance =
      (mResistance + Complex(0, omega) * mInductance).inverse() +
      Complex(0, omega) * mCapacitance / 2;
  MatrixComp cur1 =
      initAdmittance * volt1 -
      (mResistance + Complex(0, omega) * mInductance).inverse() * volt2;
  MatrixComp cur2 =
      initAdmittance * volt2 -
      (mResistance + Complex(0, omega) * mInductance).inverse() * volt1;

  SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  **mIntfCurrent = cur1.real();

  // Resize ring buffers and initialize
  mVolt1 = volt1.real().transpose().replicate(mBufSize, 1);
  mVolt2 = volt2.real().transpose().replicate(mBufSize, 1);
  mCur1 = cur1.real().transpose().replicate(mBufSize, 1);
  mCur2 = cur2.real().transpose().replicate(mBufSize, 1);
}

Matrix EMT::Ph3::DecouplingLine::interpolate(Matrix &data) {
  // linear interpolation of the nearest values
  Matrix c1 = data.row(mBufIdx);
  Matrix c2 = mBufIdx == mBufSize - 1 ? data.row(0) : data.row(mBufIdx + 1);
  return (mAlpha * c1 + (1 - mAlpha) * c2).transpose();
}

void EMT::Ph3::DecouplingLine::step(Real time, Int timeStepCount) {
  Matrix volt1 = interpolate(mVolt1);
  Matrix volt2 = interpolate(mVolt2);
  Matrix cur1 = interpolate(mCur1);
  Matrix cur2 = interpolate(mCur2);
  Matrix denom = (mSurgeImpedance + (mResistance / 4)) *
                 (mSurgeImpedance + (mResistance / 4));

  if (timeStepCount == 0) {
    // initialization
    **mSrcCur1Ref =
        cur1 - (mSurgeImpedance + mResistance / 4).inverse() * volt1;
    **mSrcCur2Ref =
        cur2 - (mSurgeImpedance + mResistance / 4).inverse() * volt2;
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

void EMT::Ph3::DecouplingLine::postStep() {
  // Update ringbuffers with new values
  mVolt1.row(mBufIdx) = -mRes1->intfVoltage().transpose();
  mVolt2.row(mBufIdx) = -mRes2->intfVoltage().transpose();
  mCur1.row(mBufIdx) =
      -mRes1->intfCurrent().transpose() + mSrcCur1->get().real().transpose();
  mCur2.row(mBufIdx) =
      -mRes2->intfCurrent().transpose() + mSrcCur2->get().real().transpose();

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;
}

void EMT::Ph3::DecouplingLine::mnaParentPreStep(Real time, Int timeStepCount) {
  step(time, timeStepCount);
  mSrc1->mnaPreStep(time, timeStepCount);
  mSrc2->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::DecouplingLine::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  postStep();
}

void EMT::Ph3::DecouplingLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mIntfVoltage = -mRes2->intfVoltage() + mRes1->intfVoltage();
}

void EMT::Ph3::DecouplingLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  **mIntfCurrent = -mRes1->intfCurrent() + mSrcCur1->get().real();
}

void EMT::Ph3::DecouplingLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mStates);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::DecouplingLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mStates);
}
