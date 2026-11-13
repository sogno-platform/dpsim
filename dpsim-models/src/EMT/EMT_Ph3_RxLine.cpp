/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_RxLine.h>
#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::RxLine::RxLine(String uid, String name, Logger::Level logLevel)
    : RxLine(uid, name, true, logLevel) {}

EMT::Ph3::RxLine::RxLine(String uid, String name, Bool useSubcomponents,
                         Logger::Level logLevel)
    : Base::Ph3::PiLine(mAttributes),
      CompositePowerComp<Real>(uid, name, true, true, logLevel) {
  mUseSubcomponents = useSubcomponents;
  mDirectState = mAttributes->create<Matrix>("direct_state");
  **mDirectState = Matrix::Zero(6, 1);
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(1);
  setTerminalNumber(2);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  mSLog->flush();
}

/// DEPRECATED: Delete method
SimPowerComp<Real>::Ptr EMT::Ph3::RxLine::clone(String name) {
  auto copy = RxLine::make(name, mUseSubcomponents, mLogLevel);
  copy->setParameters(**mSeriesRes, **mSeriesInd);
  return copy;
}

void EMT::Ph3::RxLine::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  if (!mUseSubcomponents)
    return;

  // Default model with virtual node in between
  mSubResistor =
      std::make_shared<EMT::Ph3::Resistor>(**mName + "_res", mLogLevel);
  mSubResistor->setParameters(**mSeriesRes);
  mSubResistor->connect({mTerminals[0]->node(), mVirtualNodes[0]});
  addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);

  mSubInductor =
      std::make_shared<EMT::Ph3::Inductor>(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(**mSeriesInd);
  mSubInductor->connect({mVirtualNodes[0], mTerminals[1]->node()});
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mInitialResistor =
      std::make_shared<EMT::Ph3::Resistor>(**mName + "_snubber_res", mLogLevel);
  Matrix defaultSnubRes = Matrix::Zero(3, 3);
  defaultSnubRes << 1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6;
  mInitialResistor->setParameters(defaultSnubRes);
  mInitialResistor->connect({SimNode::GND, mTerminals[1]->node()});
  addMNASubComponent(mInitialResistor,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
}

void EMT::Ph3::RxLine::initializeParentFromNodesAndTerminals(Real frequency) {
  // Match PiLine's RMS line-to-neutral power-flow phasors to the peak
  // phase-to-ground values used by the EMT subcomponents. The virtual node is
  // not initialized yet here, so deriving the branch voltage from it would
  // start the inductor with an arbitrary (usually zero) voltage.
  Real omega = 2. * PI * frequency;
  MatrixComp impedance = MatrixComp::Zero(3, 3);
  impedance << Complex((**mSeriesRes)(0, 0), omega * (**mSeriesInd)(0, 0)),
      Complex((**mSeriesRes)(0, 1), omega * (**mSeriesInd)(0, 1)),
      Complex((**mSeriesRes)(0, 2), omega * (**mSeriesInd)(0, 2)),
      Complex((**mSeriesRes)(1, 0), omega * (**mSeriesInd)(1, 0)),
      Complex((**mSeriesRes)(1, 1), omega * (**mSeriesInd)(1, 1)),
      Complex((**mSeriesRes)(1, 2), omega * (**mSeriesInd)(1, 2)),
      Complex((**mSeriesRes)(2, 0), omega * (**mSeriesInd)(2, 0)),
      Complex((**mSeriesRes)(2, 1), omega * (**mSeriesInd)(2, 1)),
      Complex((**mSeriesRes)(2, 2), omega * (**mSeriesInd)(2, 2));

  MatrixComp vInitABC = MatrixComp::Zero(3, 1);
  vInitABC(0, 0) =
      RMS3PH_TO_PEAK1PH * (initialSingleVoltage(1) - initialSingleVoltage(0));
  vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
  vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

  MatrixComp iInit = impedance.inverse() * vInitABC;
  **mIntfCurrent = iInit.real();
  **mIntfVoltage = vInitABC.real();
  // Initialization of virtual node
  // Initial voltage of phase B,C is set after A
  MatrixComp vInitTerm0 = MatrixComp::Zero(3, 1);
  vInitTerm0(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  vInitTerm0(1, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_B;
  vInitTerm0(2, 0) = vInitTerm0(0, 0) * SHIFT_TO_PHASE_C;

  mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH *
                                      (vInitTerm0 + **mSeriesRes * iInit));
  if (!mUseSubcomponents) {
    (**mDirectState).setZero();
    (**mDirectState).block<3, 1>(0, 0) = iInit.real();
    (**mDirectState).block<3, 1>(3, 0) =
        (vInitABC - (**mSeriesRes).cast<Complex>() * iInit).real();
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)));
}

void EMT::Ph3::RxLine::mnaParentInitialize(Real, Real timeStep,
                                           Attribute<Matrix>::Ptr) {
  if (!mUseSubcomponents)
    mDirectInductorAdmittance = (timeStep / 2.) * (**mSeriesInd).inverse();
}

void EMT::Ph3::RxLine::mnaParentApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  if (mUseSubcomponents)
    return;
  updateMatrixNodeIndices();
  cacheDirectMatrixIndices();
  const Matrix snubberConductance = Matrix::Identity(3, 3) * 1e-6;
  MNAStampUtils::stampConductanceMatrix(
      (**mSeriesRes).inverse(), systemMatrix, mDirectTerminalIndices[0][0],
      mDirectMidpointIndices[0], mDirectTerminalConnected[0], true, mSLog);
  MNAStampUtils::stampConductanceMatrix(
      mDirectInductorAdmittance, systemMatrix, mDirectMidpointIndices[0],
      mDirectTerminalIndices[1][0], true, mDirectTerminalConnected[1], mSLog);
  MNAStampUtils::stampConductanceMatrix(
      snubberConductance, systemMatrix, mDirectTerminalIndices[1][0], 0,
      mDirectTerminalConnected[1], false, mSLog);
}

void EMT::Ph3::RxLine::cacheDirectMatrixIndices() {
  for (UInt terminal = 0; terminal < 2; ++terminal) {
    mDirectTerminalConnected[terminal] = terminalNotGrounded(terminal);
    for (UInt phase = 0; phase < 3; ++phase)
      mDirectTerminalIndices[terminal][phase] =
          matrixNodeIndex(terminal, phase);
  }
  for (UInt phase = 0; phase < 3; ++phase)
    mDirectMidpointIndices[phase] =
        mVirtualNodes[0]->matrixNodeIndex(static_cast<PhaseType>(phase));
}

void EMT::Ph3::RxLine::mnaParentApplyRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents)
    return;
  for (UInt phase = 0; phase < 3; ++phase) {
    Math::setVectorElement(rightVector, mDirectMidpointIndices[phase],
                           mDirectHistory(phase));
    if (mDirectTerminalConnected[1])
      Math::setVectorElement(rightVector, mDirectTerminalIndices[1][phase],
                             -mDirectHistory(phase));
  }
}

Bool EMT::Ph3::RxLine::mnaHasCompactRightSideVectorStamp() const {
  return !mUseSubcomponents ||
         CompositePowerComp<Real>::mnaHasCompactRightSideVectorStamp();
}

void EMT::Ph3::RxLine::mnaAddCompactRightSideVectorStamp(Matrix &rightVector) {
  if (mUseSubcomponents) {
    CompositePowerComp<Real>::mnaAddCompactRightSideVectorStamp(rightVector);
    return;
  }
  for (UInt phase = 0; phase < 3; ++phase) {
    Math::addToVectorElement(
        rightVector, mDirectMidpointIndices[phase],
        Math::realFromVectorElement(**mRightVector,
                                    mDirectMidpointIndices[phase]));
    if (mDirectTerminalConnected[1]) {
      const UInt index = mDirectTerminalIndices[1][phase];
      Math::addToVectorElement(
          rightVector, index,
          Math::realFromVectorElement(**mRightVector, index));
    }
  }
}

void EMT::Ph3::RxLine::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  if (!mUseSubcomponents)
    prevStepDependencies.push_back(mDirectState);
  modifiedAttributes.push_back(mRightVector);
};

void EMT::Ph3::RxLine::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mIntfVoltage);
  if (!mUseSubcomponents)
    modifiedAttributes.push_back(mDirectState);
};

void EMT::Ph3::RxLine::mnaParentPreStep(Real time, Int timeStepCount) {
  if (!mUseSubcomponents) {
    mDirectHistory.noalias() =
        mDirectInductorAdmittance * (**mDirectState).block<3, 1>(3, 0);
    mDirectHistory += (**mDirectState).block<3, 1>(0, 0);
  }
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::RxLine::mnaParentPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  if (!mUseSubcomponents) {
    Eigen::Matrix<Real, 3, 1> v0 = Eigen::Matrix<Real, 3, 1>::Zero();
    Eigen::Matrix<Real, 3, 1> v1 = Eigen::Matrix<Real, 3, 1>::Zero();
    Eigen::Matrix<Real, 3, 1> vmid;
    if (mDirectTerminalConnected[0])
      for (UInt phase = 0; phase < 3; ++phase)
        v0(phase) = Math::realFromVectorElement(
            **leftVector, mDirectTerminalIndices[0][phase]);
    if (mDirectTerminalConnected[1])
      for (UInt phase = 0; phase < 3; ++phase)
        v1(phase) = Math::realFromVectorElement(
            **leftVector, mDirectTerminalIndices[1][phase]);
    for (UInt phase = 0; phase < 3; ++phase)
      vmid(phase) = Math::realFromVectorElement(**leftVector,
                                                mDirectMidpointIndices[phase]);
    **mIntfVoltage = v1 - v0;
    **mIntfCurrent = mDirectInductorAdmittance * (v1 - vmid) + mDirectHistory;
    (**mDirectState).block<3, 1>(0, 0) = **mIntfCurrent;
    (**mDirectState).block<3, 1>(3, 0) = v1 - vmid;
    return;
  }
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::RxLine::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  (**mIntfVoltage).setZero();
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}

void EMT::Ph3::RxLine::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (!mUseSubcomponents) {
    Eigen::Matrix<Real, 3, 1> v1 = Eigen::Matrix<Real, 3, 1>::Zero();
    Eigen::Matrix<Real, 3, 1> vmid;
    if (mDirectTerminalConnected[1])
      for (UInt phase = 0; phase < 3; ++phase)
        v1(phase) = Math::realFromVectorElement(
            leftVector, mDirectTerminalIndices[1][phase]);
    for (UInt phase = 0; phase < 3; ++phase)
      vmid(phase) = Math::realFromVectorElement(leftVector,
                                                mDirectMidpointIndices[phase]);
    **mIntfCurrent = mDirectInductorAdmittance * (v1 - vmid) + mDirectHistory;
    return;
  }
  **mIntfCurrent = mSubInductor->intfCurrent();
}
