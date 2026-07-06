// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_CurrentSource.h>

using namespace CPS;

DP::Ph3::CurrentSource::CurrentSource(String uid, String name,
                                      Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
      mCurrentRef(mAttributes->create<MatrixComp>("I_ref")),
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(0);
  setTerminalNumber(2);
  **mCurrentRef = MatrixComp::Zero(3, 1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::CurrentSource::clone(String name) {
  auto copy = CurrentSource::make(name, mLogLevel);
  copy->setParameters(**mCurrentRef, **mSrcFreq);
  return copy;
}

void DP::Ph3::CurrentSource::setParameters(MatrixComp currentRef,
                                           Real srcFreq) {
  auto srcSigSine = Signal::SineWaveGenerator::make(**mName + "_sw");
  srcSigSine->setParameters(Complex(1, 0), srcFreq);
  mSrcSig = srcSigSine;
  mSrcFreq->setReference(mSrcSig->mFreq);

  **mCurrentRef = currentRef;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nCurrent reference phasor [A]: {:s}"
                     "\nFrequency [Hz]: {:s}",
                     Logger::matrixCompToString(currentRef),
                     Logger::realToString(srcFreq));

  mParametersSet = true;
}

void DP::Ph3::CurrentSource::initializeFromNodesAndTerminals(Real frequency) {
  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from node voltages and terminal ---"
                     "\nReference current: {:s}",
                     Logger::matrixCompToString(**mCurrentRef));
  mSLog->flush();
}

void DP::Ph3::CurrentSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  updateCurrent(0);
}

void DP::Ph3::CurrentSource::mnaCompApplyRightSideVectorStamp(
    Matrix &rightVector) {
  if (terminalNotGrounded(1)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 0),
                           -(**mIntfCurrent)(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 1),
                           -(**mIntfCurrent)(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(1, 2),
                           -(**mIntfCurrent)(2, 0));
  }
  if (terminalNotGrounded(0)) {
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 0),
                           (**mIntfCurrent)(0, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 1),
                           (**mIntfCurrent)(1, 0));
    Math::setVectorElement(rightVector, matrixNodeIndex(0, 2),
                           (**mIntfCurrent)(2, 0));
  }
}

void DP::Ph3::CurrentSource::updateCurrent(Real time) {
  if (mSrcSig != nullptr) {
    mSrcSig->step(time);
    **mIntfCurrent = **mCurrentRef * mSrcSig->getSignal();
  } else {
    **mIntfCurrent = **mCurrentRef;
  }
}

void DP::Ph3::CurrentSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mCurrentRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::CurrentSource::mnaCompPreStep(Real time, Int timeStepCount) {
  updateCurrent(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::CurrentSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph3::CurrentSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph3::CurrentSource::mnaCompUpdateVoltage(const Matrix &leftVector) {
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
    (**mIntfVoltage)(1, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 1));
    (**mIntfVoltage)(2, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 2));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
    (**mIntfVoltage)(1, 0) =
        (**mIntfVoltage)(1, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 1));
    (**mIntfVoltage)(2, 0) =
        (**mIntfVoltage)(2, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 2));
  }
}
