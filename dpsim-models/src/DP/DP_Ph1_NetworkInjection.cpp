/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_NetworkInjection.h>

using namespace CPS;

DP::Ph1::NetworkInjection::NetworkInjection(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	mVoltageRef(mAttributes->createDynamic<Complex>("V_ref")),
	mSrcFreq(mAttributes->createDynamic<Real>("f_src")) {
	setVirtualNodeNumber(0);
	setTerminalNumber(1);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	**mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);

	// Create electrical sub components
	mSubVoltageSource = std::make_shared<DP::Ph1::VoltageSource>(**mName + "_vs", mLogLevel);
	addMNASubComponent(mSubVoltageSource, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	SPDLOG_LOGGER_INFO(mSLog, "Electrical subcomponents: ");
	for (auto subcomp: mSubComponents)
		SPDLOG_LOGGER_INFO(mSLog, "- {}", subcomp->name());

	mSubVoltageSource->mVoltageRef->setReference(mVoltageRef);
	mSubVoltageSource->mSrcFreq->setReference(mSrcFreq);
}

SimPowerComp<Complex>::Ptr DP::Ph1::NetworkInjection::clone(String name) {
	auto copy = NetworkInjection::make(name, mLogLevel);
	copy->setParameters(**mVoltageRef);
	return copy;
}

void DP::Ph1::NetworkInjection::setParameters(Complex voltageRef, Real srcFreq) {
	mParametersSet = true;

	mSubVoltageSource->setParameters(voltageRef, srcFreq);

	SPDLOG_LOGGER_INFO(mSLog, "\nVoltage Ref={:s} [V]"
				"\nFrequency={:s} [Hz]",
				Logger::phasorToString(voltageRef),
				Logger::realToString(srcFreq));
}

void DP::Ph1::NetworkInjection::setParameters(Complex initialPhasor, Real freqStart, Real rocof, Real timeStart, Real duration, bool smoothRamp) {
	mParametersSet = true;

	mSubVoltageSource->setParameters(initialPhasor, freqStart, rocof, timeStart, duration, smoothRamp);

	SPDLOG_LOGGER_INFO(mSLog, "\nVoltage Ref={:s} [V]"
				"\nFrequency={:s} [Hz]",
				Logger::phasorToString(initialPhasor),
				Logger::realToString(freqStart));
}

void DP::Ph1::NetworkInjection::setParameters(Complex initialPhasor, Real modulationFrequency, Real modulationAmplitude, Real baseFrequency /*= 0.0*/, bool zigzag /*= false*/) {
	mParametersSet = true;

	mSubVoltageSource->setParameters(initialPhasor, modulationFrequency, modulationAmplitude, baseFrequency, zigzag);

	SPDLOG_LOGGER_INFO(mSLog, "\nVoltage Ref={:s} [V]"
				"\nFrequency={:s} [Hz]",
				Logger::phasorToString(initialPhasor),
				Logger::realToString(baseFrequency));
}

void DP::Ph1::NetworkInjection::initializeFromNodesAndTerminals(Real frequency) {
	// Connect electrical subcomponents
	mSubVoltageSource->connect({ SimNode::GND, node(0) });

	// Initialize electrical subcomponents
	for (auto subcomp: mSubComponents) {
		subcomp->initialize(mFrequencies);
		subcomp->initializeFromNodesAndTerminals(frequency);
	}
}

// #### MNA functions ####

void DP::Ph1::NetworkInjection::mnaParentApplyRightSideVectorStamp(Matrix& rightVector) {
	SPDLOG_LOGGER_DEBUG(mSLog, "Right Side Vector: {:s}",
				Logger::matrixToString(rightVector));
}

void DP::Ph1::NetworkInjection::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::NetworkInjection::mnaParentPreStep(Real time, Int timeStepCount) {
	// pre-step of component itself
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::NetworkInjection::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::NetworkInjection::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of component itself
	mnaCompUpdateCurrent(**leftVector);
	mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph1::NetworkInjection::mnaCompUpdateVoltage(const Matrix& leftVector) {
	**mIntfVoltage = **mSubVoltageSource->mIntfVoltage;
}

void DP::Ph1::NetworkInjection::mnaCompUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = **mSubVoltageSource->mIntfCurrent;
}