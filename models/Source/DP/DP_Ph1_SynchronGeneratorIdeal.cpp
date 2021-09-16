/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_SynchronGeneratorIdeal.h>

using namespace CPS;


DP::Ph1::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String uid, String name,
	Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(1);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("V_ref", &mVoltageRef, Flags::read);
}

DP::Ph1::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String name,
	Logger::Level logLevel)
	: SynchronGeneratorIdeal(name, name, logLevel) { }

SimPowerComp<Complex>::Ptr DP::Ph1::SynchronGeneratorIdeal::clone(String name) {
	return SynchronGeneratorIdeal::make(name, mLogLevel);
}

void DP::Ph1::SynchronGeneratorIdeal::initializeFromNodesAndTerminals(Real frequency) {
	mSubVoltageSource = DP::Ph1::VoltageSource::make(mName + "_src", mLogLevel);
	mSubVoltageSource->connect({ SimNode::GND, node(0) });
	mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubVoltageSource->initialize(mFrequencies);
	mSubVoltageSource->initializeFromNodesAndTerminals(frequency);

	setAttributeRef("V_ref", mSubVoltageSource->attribute<Complex>("V_ref"));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)));
}

void DP::Ph1::SynchronGeneratorIdeal::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mSubVoltageSource->mnaInitialize(omega, timeStep, leftVector);
	// since there is no additional behaviour, just use the tasks and right-vector from the voltage source
	setAttributeRef("right_vector", mSubVoltageSource->attribute("right_vector"));
	for (auto task : mSubVoltageSource->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
}

void DP::Ph1::SynchronGeneratorIdeal::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubVoltageSource->mnaApplyRightSideVectorStamp(rightVector);
}

