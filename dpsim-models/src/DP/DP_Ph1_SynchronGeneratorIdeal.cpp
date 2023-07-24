/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_SynchronGeneratorIdeal.h>

using namespace CPS;


DP::Ph1::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String uid, String name,
	Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	mVoltageRef(mAttributes->createDynamic<Complex>("V_ref")) {
	setVirtualNodeNumber(1);
	setTerminalNumber(1);
	**mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);
}

DP::Ph1::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String name,
	Logger::Level logLevel)
	: SynchronGeneratorIdeal(name, name, logLevel) { }

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGeneratorIdeal::clone(String name) {
	return SynchronGeneratorIdeal::make(name, mLogLevel);
}

void DP::Ph1::SynchronGeneratorIdeal::initializeFromNodesAndTerminals(Real frequency) {
	mSubVoltageSource = DP::Ph1::VoltageSource::make(**mName + "_src", mLogLevel);
	mSubVoltageSource->mVoltageRef->setReference(mVoltageRef);
	mSubVoltageSource->connect({ SimNode::GND, node(0) });
	mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubVoltageSource->initialize(mFrequencies);
	mSubVoltageSource->initializeFromNodesAndTerminals(frequency);
	addMNASubComponent(mSubVoltageSource, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

	SPDLOG_LOGGER_DEBUG(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)));
}


void DP::Ph1::SynchronGeneratorIdeal::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaParentPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
	mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaCompUpdateCurrent(const Matrix& leftvector) {
	**mIntfCurrent = **mSubComponents[0]->mIntfCurrent;
}

void DP::Ph1::SynchronGeneratorIdeal::mnaCompUpdateVoltage(const Matrix& leftVector) {
	**mIntfVoltage = **mSubComponents[0]->mIntfVoltage;
}
