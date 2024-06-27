/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_CurrentSource.h>

using namespace CPS;

EMT::Ph3::CurrentSource::CurrentSource(String uid, String name,
                                       Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mCurrentRef(mAttributes->create<MatrixComp>("I_ref")), // rms-value
      mSrcFreq(mAttributes->createDynamic<Real>("f_src")),
      mSigOut(mAttributes->createDynamic<Complex>("sigOut")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(0);
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);
}
SimPowerComp<Real>::Ptr EMT::Ph3::CurrentSource::clone(String name) {
	auto copy = CurrentSource::make(name, mLogLevel);
	// TODO: implement setParameters
	// copy->setParameters(attributeTyped<MatrixComp>("I_ref")->get(), attributeTyped<Real>("f_src")->get());
	return copy;
}

void EMT::Ph3::CurrentSource::setParameters(MatrixComp currentRef, Real srcFreq)
{
	auto srcSigSine = Signal::SineWaveGenerator::make(**mName + "_sw");
	// Complex(1,0) is used as initialPhasor for signal generator as only phase is used
	srcSigSine->setParameters(Complex(1,0), srcFreq);
	mSrcSig = srcSigSine;

	**mCurrentRef = currentRef;
	mSrcFreq->setReference(mSrcSig->mFreq);

	mSLog->info("\nCurrent reference phasor [I]: {:s}"
				"\nFrequency [Hz]: {:s}",
				Logger::matrixCompToString(currentRef),
				Logger::realToString(srcFreq));
				
	mParametersSet = true;
}

void EMT::Ph3::CurrentSource::initializeFromNodesAndTerminals(Real frequency) {
  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminal ---");
  if (!mParametersSet) {
    auto srcSigSine =
        Signal::SineWaveGenerator::make(**mName + "_sw", Logger::Level::off);
    // Complex(1,0) is used as initialPhasor for signal generator as only phase is used
    srcSigSine->setParameters(Complex(1, 0), frequency);
    mSrcSig = srcSigSine;

    Complex v_ref = initialSingleVoltage(1) - initialSingleVoltage(0);
    Complex s_ref = terminal(1)->singlePower() - terminal(0)->singlePower();

    // Current flowing from T1 to T0 (rms value)
    Complex i_ref = std::conj(s_ref / v_ref / sqrt(3.));

    **mCurrentRef = CPS::Math::singlePhaseVariableToThreePhase(i_ref);
    mSrcFreq->setReference(mSrcSig->attributeTyped<Real>("freq"));

    SPDLOG_LOGGER_INFO(mSLog,
                       "\nReference current: {:s}"
                       "\nReference voltage: {:s}"
                       "\nReference power: {:s}"
                       "\nTerminal 0 voltage: {:s}"
                       "\nTerminal 1 voltage: {:s}"
                       "\nTerminal 0 power: {:s}"
                       "\nTerminal 1 power: {:s}",
                       Logger::phasorToString(i_ref),
                       Logger::phasorToString(v_ref),
                       Logger::complexToString(s_ref),
                       Logger::phasorToString(initialSingleVoltage(0)),
                       Logger::phasorToString(initialSingleVoltage(1)),
                       Logger::complexToString(terminal(0)->singlePower()),
                       Logger::complexToString(terminal(1)->singlePower()));
  } else {
    SPDLOG_LOGGER_INFO(
        mSLog,
        "\nInitialization from node voltages and terminal omitted (parameter "
        "already set)."
        "\nReference voltage: {:s}",
        Logger::matrixCompToString(attributeTyped<MatrixComp>("I_ref")->get()));
  }
  SPDLOG_LOGGER_INFO(
      mSLog, "\n--- Initialization from node voltages and terminal ---");
  mSLog->flush();
}

void EMT::Ph3::CurrentSource::mnaCompInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
}

void EMT::Ph3::CurrentSource::mnaCompApplyRightSideVectorStamp(
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

void EMT::Ph3::CurrentSource::updateCurrent(Real time) {
  if (mSrcSig != nullptr) {
    mSrcSig->step(time);
    for (int i = 0; i < 3; i++) {
      (**mIntfCurrent)(i, 0) = RMS_TO_PEAK * Math::abs((**mCurrentRef)(i, 0)) *
                               cos(Math::phase(mSrcSig->getSignal()) +
                                   Math::phase((**mCurrentRef)(i, 0)));
    }
  } else {
    **mIntfCurrent = RMS_TO_PEAK * (**mCurrentRef).real();
  }
  SPDLOG_LOGGER_DEBUG(mSLog, "\nUpdate current: {:s}",
                      Logger::matrixToString(**mIntfCurrent));
}

void EMT::Ph3::CurrentSource::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mCurrentRef);
  modifiedAttributes.push_back(mRightVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph3::CurrentSource::mnaCompPreStep(Real time, Int timeStepCount) {
  updateCurrent(time);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::CurrentSource::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
};

void EMT::Ph3::CurrentSource::mnaCompPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::CurrentSource::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // v1 - v0
  **mIntfVoltage = Matrix::Zero(3, 1);
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