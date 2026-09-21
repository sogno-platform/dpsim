// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include "dpsim-models/Definitions.h"
#include "dpsim-models/TopologicalNode.h"
#include <dpsim-models/Signal/DecouplingIdealTransformer_EMT_Ph3.h>

using namespace CPS;
using namespace CPS::EMT::Ph3;
using namespace CPS::Signal;

DecouplingIdealTransformer_EMT_Ph3::DecouplingIdealTransformer_EMT_Ph3(
    String uid, String name, Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mStates(mAttributes->create<Matrix>("states")),
      mSourceVoltageIntfVoltage(
          mAttributes->create<Matrix>("v_src_intf", Matrix::Zero(3, 1))),
      mSourceVoltageIntfCurrent(
          mAttributes->create<Matrix>("i_src_intf", Matrix::Zero(3, 1))),
      mSrcVoltageRef(mAttributes->create<Matrix>("v_ref", Matrix::Zero(3, 1))),
      mSrcCurrentRef(mAttributes->create<Matrix>("i_ref", Matrix::Zero(3, 1))) {

  mPhaseType = PhaseType::ABC;
  setTerminalNumber(2);
  setVirtualNodeNumber(1);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

void DecouplingIdealTransformer_EMT_Ph3::setParameters(
    Real delay, Matrix voltageSrcIntfCurr, Matrix current1Extrap0,
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

void DecouplingIdealTransformer_EMT_Ph3::createSubComponents() {
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
  mRes2->connect({mTerminals[1]->node(), EMT::SimNode::GND});
  addMNASubComponent(mRes2, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mVoltageSrc = ControlledVoltageSource::make(**mName + "_v", mLogLevel);
  mVoltageSrc->setParameters(Matrix::Zero(3, 1));
  mVoltageSrc->connect({mVirtualNodes[0], EMT::SimNode::GND});
  addMNASubComponent(mVoltageSrc, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mCurrentSrc = ControlledCurrentSource::make(**mName + "_i", mLogLevel);
  mCurrentSrc->setParameters(Matrix::Zero(3, 1));
  mCurrentSrc->connect({mTerminals[1]->node(), EMT::SimNode::GND});
  addMNASubComponent(mCurrentSrc, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSrcVoltage = mVoltageSrc->mVoltageRef;
  mSrcCurrent = mCurrentSrc->mCurrentRef;
}

void DecouplingIdealTransformer_EMT_Ph3::initializeParentFromNodesAndTerminals(
    Real frequency) {

  MatrixComp cur1 = mVoltageSrcIntfCurr.cast<Complex>();
  MatrixComp volt2 = initialVoltage(1);

  mVirtualNodes[0]->setInitialVoltage(initialVoltage(0) -
                                      mInternalSeriesResistance * cur1);

  SPDLOG_LOGGER_INFO(mSLog, "initial current: i_1 {}", cur1);
  SPDLOG_LOGGER_INFO(mSLog, "initial voltage: v_2 {}", volt2);

  **mSrcVoltageRef = volt2.real();
  **mSrcCurrentRef = cur1.real();
  mVoltageSrc->setParameters(**mSrcVoltageRef);
  mCurrentSrc->setParameters(**mSrcCurrentRef);
}

void DecouplingIdealTransformer_EMT_Ph3::mnaParentInitialize(
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

  Matrix cur1 = **mSrcCurrentRef;
  Matrix volt2 = **mSrcVoltageRef;

  mCurrentSrc->setIntfVoltage(volt2);

  mVoltageSrc->setIntfVoltage(volt2);
  mVoltageSrc->setIntfCurrent(mVoltageSrcIntfCurr);

  **mSourceVoltageIntfVoltage = volt2;
  **mSourceVoltageIntfCurrent = mVoltageSrc->intfCurrent();

  **mIntfVoltage = volt2;
  **mIntfCurrent = cur1;

  // Resize ring buffers and initialize
  mCur1 = cur1.transpose().replicate(mBufSize, 1);
  mVol2 = volt2.transpose().replicate(mBufSize, 1);

  SPDLOG_LOGGER_INFO(mSLog, "Verify initial current: i_1 {}",
                     mCurrentSrc->intfCurrent()(0, 0));
  SPDLOG_LOGGER_INFO(mSLog, "Verify initial voltage: v_2 {}",
                     mVoltageSrc->intfVoltage()(0, 0));

  mCur1Extrap = Matrix(mExtrapolationDegree + 1, 3);
  mCur1Extrap.row(0) = mCurrent1Extrap0.real().transpose();
  if (mExtrapolationDegree > 0) {
    mCur1Extrap.row(1) = mVoltageSrcIntfCurr.transpose();
  }
  mVol2Extrap = volt2.transpose().replicate(mExtrapolationDegree + 1, 1);
}

Matrix DecouplingIdealTransformer_EMT_Ph3::interpolate(Matrix &data) {
  Matrix c1 = data.row(mBufIdx);
  Matrix c2 = mBufIdx == mBufSize - 1 ? data.row(0) : data.row(mBufIdx + 1);
  return (mAlpha * c1 + (1 - mAlpha) * c2).transpose();
}

Matrix DecouplingIdealTransformer_EMT_Ph3::extrapolate(Matrix &data) {
  if (mCouplingMethod == CouplingMethod::EXTRAPOLATION_LINEAR) {
    Matrix c1 = data.row(mMacroBufIdx);
    Matrix c2 = mMacroBufIdx == mExtrapolationDegree
                    ? data.row(0)
                    : data.row(mMacroBufIdx + 1);
    Real delayFraction =
        (mDelay * (mBufIdx + 1)) / static_cast<float>(mBufSize);
    Real tEval = mDelay + delayFraction;
    return (((c2 - c1) / mDelay) * tEval + c1).transpose();
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

void DecouplingIdealTransformer_EMT_Ph3::postStep() {
  // Update ringbuffers with new values
  mCur1.row(mBufIdx) = -mVoltageSrc->intfCurrent().transpose();
  mVol2.row(mBufIdx) = mCurrentSrc->intfVoltage().transpose();

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

void DecouplingIdealTransformer_EMT_Ph3::mnaParentPreStep(Real time,
                                                          Int timeStepCount) {
  step(time, timeStepCount);
  mVoltageSrc->mnaPreStep(time, timeStepCount);
  mCurrentSrc->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DecouplingIdealTransformer_EMT_Ph3::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  postStep();
}

void DecouplingIdealTransformer_EMT_Ph3::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  **mIntfVoltage = mVoltageSrc->intfVoltage();
}

void DecouplingIdealTransformer_EMT_Ph3::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  **mIntfCurrent = mVoltageSrc->intfCurrent();
}

void DecouplingIdealTransformer_EMT_Ph3::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mStates);
  modifiedAttributes.push_back(mRightVector);
}

void DecouplingIdealTransformer_EMT_Ph3::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mStates);
}
