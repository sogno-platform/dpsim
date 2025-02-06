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
      mSrcCtrledCurrent(mAttributes->create<Matrix>("i_src_ctrl", Matrix::Zero(3, 1))),
      mSrcRes(mAttributes->create<Matrix>("src_res", Matrix::Zero(3, 3))),
      mReceivingVolt(mAttributes->createDynamic<Matrix>("receiving_volt")),
      mReceivingCur(mAttributes->createDynamic<Matrix>("receiving_cur")),
      mSendingVolt(mAttributes->create<Matrix>("sending_volt")),
      mSendingCur(mAttributes->create<Matrix>("sending_cur")) {

  mSubRes = EMT::Ph3::Resistor::make(**mName + "_r", mLogLevel);
  addMNASubComponent(mSubRes, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);


  mSubCtrledCurrentSource = EMT::Ph3::ControlledCurrentSource::make(name + "_i", logLevel);
  // Pre-step of the subcontrolled current source is handled explicitly in mnaParentPreStep
  addMNASubComponent(mSubCtrledCurrentSource, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  // addMNASubComponent(mSubCtrledCurrentSource, MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT,
  //                    MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  

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

  mSurgeImpedance << sqrt(inductance(0,0) / capacitance(0,0)), 0, 0,
                      0, sqrt(inductance(1,1) / capacitance(1,1)), 0,
                      0, 0, sqrt(inductance(2,2) / capacitance(2,2));

  mDelay << sqrt(inductance(0,0) * capacitance(0,0)), 0, 0,
            0, sqrt(inductance(1,1) * capacitance(1,1)), 0,
            0, 0, sqrt(inductance(2,2) * capacitance(2,2));

   **mSrcRes << (mSurgeImpedance(0,0) + mResistance(0,0) / 4), 0, 0,
                0, (mSurgeImpedance(1,1) + mResistance(1,1) / 4), 0,
                0, 0, (mSurgeImpedance(2,2) + mResistance(2,2) / 4);

  SPDLOG_LOGGER_INFO(mSLog, "surge impedance: {}", mSurgeImpedance);
  SPDLOG_LOGGER_INFO(mSLog, "delay: {}", mDelay);

  mSubCtrledCurrentSource->setParameters(**mSrcCtrledCurrent);
  mSubCtrledCurrentSource->connect({mTerminals[0]->node(), SimNode::GND});
  mSubRes->setParameters(**mSrcRes);
  mSubRes->connect({mTerminals[0]->node(), SimNode::GND});

  mParametersSet = true;
}

void EMT::Ph3::HalfDecouplingLine::initializeFromNodesAndTerminals(Real frequency) {
  // if (mDelay < timeStep)
  //   throw SystemError("Timestep too large for decoupling");

  // mBufSize = static_cast<UInt>(ceil(mDelay / timeStep));
  // mAlpha = 1 - (mBufSize - mDelay / timeStep);
  // SPDLOG_LOGGER_INFO(mSLog, "bufsize {} alpha {}", mBufSize, mAlpha);

  // Initialization based on static PI-line model
  MatrixComp vInit = MatrixComp::Zero(3, 1);
  vInit(0, 0) = initialSingleVoltage(0); // rms value
  vInit(1, 0) = vInit(0, 0) * SHIFT_TO_PHASE_B;
  vInit(2, 0) = vInit(0, 0) * SHIFT_TO_PHASE_C;

  MatrixComp iInit = MatrixComp::Zero(3, 1);
  // Complex sInit = terminal(0)->singlePower(); // returns zero

  // iInit(0, 0) = std::conj(sInit / vInit(0, 0) / sqrt(3.));  // rms value
  // iInit(1, 0) = std::conj(sInit / vInit(1, 0) / sqrt(3.));
  // iInit(2, 0) = std::conj(sInit / vInit(2, 0) / sqrt(3.));

  **mIntfVoltage = vInit.real();
  **mIntfCurrent = iInit.real();

  // Initialize electrical subcomponents
  for (auto subcomp : mSubComponents) {
    subcomp->initialize(mFrequencies);
    subcomp->initializeFromNodesAndTerminals(frequency);
  }

  **mSendingVolt= **mIntfVoltage;
  **mSendingCur= **mIntfCurrent;

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
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
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
}

void EMT::Ph3::HalfDecouplingLine::Step(Real time) { 

  **mSendingVolt= **mIntfVoltage;
  **mSendingCur= **mIntfCurrent;

  Matrix denom = Matrix::Zero(3,3);
  
  denom << (mSurgeImpedance(0,0) + mResistance(0,0) / 4) * (mSurgeImpedance(0,0) + mResistance(0,0) / 4), 0, 0,
            0, (mSurgeImpedance(1,1) + mResistance(1,1) / 4) * (mSurgeImpedance(1,1) + mResistance(1,1) / 4), 0,
            0, 0, (mSurgeImpedance(2,2) + mResistance(2,2) / 4) * (mSurgeImpedance(2,2) + mResistance(2,2) / 4);


  // Calculate current
  (**mSrcCtrledCurrent)(0,0) = -mSurgeImpedance(0,0) / denom(0,0) *
                      ((**mReceivingVolt)(0,0) + (mSurgeImpedance(0,0) - mResistance(0,0) / 4) * (**mReceivingCur)(0,0)) -
                  mResistance(0,0) / 4 / denom(0,0) *
                      ((**mSendingVolt)(0,0) + (mSurgeImpedance(0,0) - mResistance(0,0) / 4) * (**mSendingVolt)(0,0));
                      
  (**mSrcCtrledCurrent)(1,0) = -mSurgeImpedance(1,1) / denom(1,1) *
                      ((**mReceivingVolt)(1,0) + (mSurgeImpedance(1,1) - mResistance(1,1) / 4) * (**mReceivingCur)(1,0)) -
                  mResistance(1,1) / 4 / denom(1,1) *
                      ((**mSendingVolt)(1,0) + (mSurgeImpedance(1,1) - mResistance(1,1) / 4) * (**mSendingVolt)(1,0));
                      
  (**mSrcCtrledCurrent)(2,0) = -mSurgeImpedance(2,2) / denom(2,2) *
                      ((**mReceivingVolt)(2,0) + (mSurgeImpedance(2,2) - mResistance(2,2) / 4) * (**mReceivingCur)(2,0)) -
                  mResistance(2,2) / 4 / denom(2,2) *
                      ((**mSendingVolt)(2,0) + (mSurgeImpedance(2,2) - mResistance(2,2) / 4) * (**mSendingVolt)(2,0));

  mSubCtrledCurrentSource->mCurrentRef->set(**mSrcCtrledCurrent);
}

void EMT::Ph3::HalfDecouplingLine::mnaParentPreStep(Real time, Int timeStepCount) {
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