/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_HalfDecouplingLine.h>

using namespace CPS;

EMT::Ph3::HalfDecouplingLine::HalfDecouplingLine(String uid, String name, Logger::Level logLevel)
    :CompositePowerComp<Real>(uid, name, true, true, logLevel),
      mSrcCtrledCurrent(mAttributes->create<Matrix>("i_src_ctrl")),
      mReceivingVolt(mAttributes->createDynamic<Matrix>("receiving_volt")),
      mReceivingCur(mAttributes->createDynamic<Matrix>("receiving_cur")),
      mSendingVoltage(mAttributes->create<Matrix>("sending_volt")),
      mSendingCur(mAttributes->create<Matrix>("sending_cur")) {

  mSubRes = EMT::Ph3::Resistor::make(**mName + "_r", mLogLevel);
  addMNASubComponent(mSubRes, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);


  mSubCtrledCurrentSource = EMT::Ph3::ControlledCurrentSource::make(name + "_i", logLevel);
  // Pre-step of the subcontrolled current source is handled explicitly in mnaParentPreStep
  addMNASubComponent(mSubCtrledCurrentSource, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  
  mSrcCtrledCurrent->setReference(mSubCtrledCurrentSource->mCurrentRef);

  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(0);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}

void EMT::Ph3::HalfDecouplingLine::setParameters(
  Matrix resistance, Matrix inductance, Matrix capacitance,
  Attribute<Matrix>::Ptr ReceivingVoltRef, Attribute<Matrix>::Ptr ReceivingCurRef) {

  mReceivingVolt->setReference(ReceivingVoltRef);
  mReceivingCur->setReference(ReceivingCurRef);

  mResistance = resistance;
  mInductance = inductance;
  mCapacitance = capacitance;
  mSurgeImpedance = sqrt(inductance(0,0) / capacitance(0,0));

  mDelay = sqrt(inductance(0,0) * capacitance(0,0));

  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mSubCtrledCurrentSource->setParameters(Matrix::Zero(3, 1));
  mSubCtrledCurrentSource->connect({mTerminals[0]->node(), SimNode::GND});
  mSubRes->setParameters(Math::singlePhaseParameterToThreePhase(mSurgeImpedance + mResistance(0,0) / 4));
  mSubRes->connect({mTerminals[0]->node(), SimNode::GND});

  mParametersSet = true;
}

void EMT::Ph3::HalfDecouplingLine::initializeFromNodesAndTerminals(Real frequency) {
  // if (mDelay < timeStep)
  //   throw SystemError("Timestep too large for decoupling");

  // mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  // mAlpha = 1 - (mBufSize - mDelay / timeStep);
  // SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // // Initialization based on static PI-line model
  // Complex volt1 = mNode1->initialSingleVoltage();
  // Complex volt2 = mNode2->initialSingleVoltage();
  // Complex initAdmittance = 1. / Complex(mResistance, omega * mInductance) +
  //                          Complex(0, omega * mCapacitance / 2);
  // Complex cur1 = volt1 * initAdmittance -
  //                volt2 / Complex(mResistance, omega * mInductance);
  // Complex cur2 = volt2 * initAdmittance -
  //                volt1 / Complex(mResistance, omega * mInductance);
  // SPDLOG_LOGGER_INFO(mSLog, "initial voltages: v_k {} v_m {}", volt1, volt2);
  // SPDLOG_LOGGER_INFO(mSLog, "initial currents: i_km {} i_mk {}", cur1, cur2);

  // // Resize ring buffers and initialize
  // mVoltBuf.resize(mBufSize, volt1.real());
  // mCurBuf.resize(mBufSize, cur1.real());
}

void EMT::Ph3::HalfDecouplingLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfVoltage);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph3::HalfDecouplingLine::Step(Real time) {
  // calculate 

  // Real denom =
  //     (mSurgeImpedance + mResistance / 4) * (mSurgeImpedance + mResistance / 4);

  // // Calculate current
  // **mSrcCurRef = -mSurgeImpedance / denom *
  //                     (**mReceivingVolt + (mSurgeImpedance - mResistance / 4) * **mReceivingCur) -
  //                 mResistance / 4 / denom *
  //                     (**mSendingVoltage + (mSurgeImpedance - mResistance / 4) * **mSendingVoltage);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentPreStep(Real time, Int timeStepCount) {
  // // Pre-step of the subcontrolled current source is handled explicitly in mnaParentPreStep
  // addMNASubComponent(mSubCtrledCurrentSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
  //                    MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  // // pre-step of subcomponents - controlled source
  // std::dynamic_pointer_cast<MNAInterface>(mSubCtrledCurrentSource)
  //     ->mnaPreStep(time, timeStepCount);
  
  // pre-step of composite component
  Step(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::HalfDecouplingLine::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  SPDLOG_LOGGER_DEBUG(mSLog, "Read voltage from {:d}", matrixNodeIndex(0));
  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::HalfDecouplingLine::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  SPDLOG_LOGGER_DEBUG(mSLog, "Read current from {:d}", matrixNodeIndex(0));
  **mIntfCurrent = **mSubRes->mIntfCurrent + **mSubCtrledCurrentSource->mIntfCurrent;
}