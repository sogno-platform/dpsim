// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_HalfDecouplingLine.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

EMT::Ph3::HalfDecouplingLine::HalfDecouplingLine(String uid, String name,
                                                 Logger::Level logLevel)
    : CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mSrcCtrledCurrent(
          mAttributes->create<Matrix>("i_src_ctrl", Matrix::Zero(3, 1))),
      mSrcRes(mAttributes->create<Matrix>("src_res", Matrix::Zero(3, 3))),
      mReceivingVolt(mAttributes->createDynamic<Matrix>("receiving_volt")),
      mReceivingCur(mAttributes->createDynamic<Matrix>("receiving_cur")),
      mSendingVolt(
          mAttributes->create<Matrix>("sending_volt", Matrix::Zero(3, 1))),
      mSendingCur(
          mAttributes->create<Matrix>("sending_cur", Matrix::Zero(3, 1))) {

  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(0);
  setTerminalNumber(1);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::HalfDecouplingLine::setParameters(Matrix resistance,
                                                 Matrix inductance,
                                                 Matrix capacitance) {

  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;

  mSurgeImpedance = (inductance * capacitance.inverse()).array().sqrt();
  mDelay = (inductance.array() * capacitance.array()).sqrt().maxCoeff();

  **mSrcRes = mSurgeImpedance + mResistance / 4;

  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mParametersSet = true;
}

void EMT::Ph3::HalfDecouplingLine::setCouplingSource(
    Attribute<Matrix>::Ptr receivingVolt, Attribute<Matrix>::Ptr receivingCur) {
  mReceivingVolt->setReference(receivingVolt);
  mReceivingCur->setReference(receivingCur);
}

void EMT::Ph3::HalfDecouplingLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  mSubRes = EMT::Ph3::Resistor::make(**mName + "_r", mLogLevel);
  mSubRes->setParameters(**mSrcRes);
  /* As in DecouplingLineEMT_Ph3, the terminating resistor is connected from
     GND to the terminal, since the Ph3 resistor has the opposite sign
     convention for voltage and current compared to its Ph1 counterpart. */
  mSubRes->connect({SimNode::GND, mTerminals[0]->node()});
  addMNASubComponent(mSubRes, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubCtrledCurrentSource =
      EMT::Ph3::ControlledCurrentSource::make(**mName + "_i", mLogLevel);
  mSubCtrledCurrentSource->setParameters(**mSrcCtrledCurrent);
  mSubCtrledCurrentSource->connect({mTerminals[0]->node(), SimNode::GND});
  addMNASubComponent(mSubCtrledCurrentSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void EMT::Ph3::HalfDecouplingLine::initializeParentFromNodesAndTerminals(
    Real frequency) {

  **mIntfVoltage = initialVoltage(0).real();
  **mIntfCurrent = Matrix::Zero(3, 1);

  // The sending quantities follow the internal sign convention of
  // DecouplingLineEMT_Ph3, in which the recorded voltage is the negated
  // terminal voltage.
  **mSendingVolt = -(**mIntfVoltage);
  **mSendingCur = Matrix::Zero(3, 1);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  if (mDelay < timeStep)
    throw SystemError("Timestep too large for decoupling");

  mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  mAlpha = 1 - (mBufSize - mDelay / timeStep);
  SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // Initialization based on static PI-line model. The far end voltage is read
  // from the coupling source, which the other half published during its own
  // initializeParentFromNodesAndTerminals.
  MatrixComp voltNear = -initialVoltage(0);
  MatrixComp voltFar = (**mReceivingVolt).cast<Complex>();

  MatrixComp seriesAdmittance =
      (mResistance + Complex(0, omega) * mInductance).inverse();
  MatrixComp initAdmittance =
      seriesAdmittance + Complex(0, omega) * mCapacitance / 2;
  MatrixComp curNear = initAdmittance * voltNear - seriesAdmittance * voltFar;

  SPDLOG_LOGGER_INFO(mSLog, "initial voltage: v_k {}", voltNear);
  SPDLOG_LOGGER_INFO(mSLog, "initial current: i_k {}", curNear);

  // Resize ring buffers and initialize
  mVoltBuf = voltNear.real().transpose().replicate(mBufSize, 1);
  mCurBuf = curNear.real().transpose().replicate(mBufSize, 1);

  **mSendingVolt = voltNear.real();
  **mSendingCur = curNear.real();
}

Matrix EMT::Ph3::HalfDecouplingLine::interpolate(Matrix &data) {
  // linear interpolation of the nearest values
  Matrix c1 = data.row(mBufIdx);
  Matrix c2 = mBufIdx == mBufSize - 1 ? data.row(0) : data.row(mBufIdx + 1);
  return (mAlpha * c1 + (1 - mAlpha) * c2).transpose();
}

void EMT::Ph3::HalfDecouplingLine::step(Real time, Int timeStepCount) {
  Matrix voltNear = **mSendingVolt;
  Matrix curNear = **mSendingCur;
  Matrix voltFar = **mReceivingVolt;
  Matrix curFar = **mReceivingCur;

  Matrix denom =
      (mSurgeImpedance + mResistance / 4) * (mSurgeImpedance + mResistance / 4);

  if (timeStepCount == 0) {
    **mSrcCtrledCurrent =
        curNear - (mSurgeImpedance + mResistance / 4).inverse() * voltNear;
  } else {
    **mSrcCtrledCurrent =
        -mSurgeImpedance * denom.inverse() *
            (voltFar + (mSurgeImpedance - mResistance / 4) * curFar) -
        mResistance / 4 * denom.inverse() *
            (voltNear + (mSurgeImpedance - mResistance / 4) * curNear);
  }

  mSubCtrledCurrentSource->mCurrentRef->set(**mSrcCtrledCurrent);
}

void EMT::Ph3::HalfDecouplingLine::postStep() {
  mVoltBuf.row(mBufIdx) = -mSubRes->intfVoltage().transpose();
  mCurBuf.row(mBufIdx) =
      -mSubRes->intfCurrent().transpose() + (**mSrcCtrledCurrent).transpose();

  mBufIdx++;
  if (mBufIdx == mBufSize)
    mBufIdx = 0;

  // Publish this end's quantities as of one travel time ago. Doing it here
  // rather than in the pre-step is what keeps the two halves free of any
  // same-step dependency on each other, so they can be solved separately.
  **mSendingVolt = interpolate(mVoltBuf);
  **mSendingCur = interpolate(mCurBuf);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentPreStep(Real time,
                                                    Int timeStepCount) {
  step(time, timeStepCount);
  mSubCtrledCurrentSource->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  postStep();
}

void EMT::Ph3::HalfDecouplingLine::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::HalfDecouplingLine::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  **mIntfCurrent = -mSubRes->intfCurrent() + **mSrcCtrledCurrent;
}

void EMT::Ph3::HalfDecouplingLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  prevStepDependencies.push_back(mReceivingVolt);
  prevStepDependencies.push_back(mReceivingCur);
  modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mSendingVolt);
  modifiedAttributes.push_back(mSendingCur);
}
