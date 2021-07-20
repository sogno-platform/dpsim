/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SynchronGeneratorIdeal.h>

using namespace CPS;


EMT::Ph3::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String uid, String name,
	Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(1);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<MatrixComp>("V_ref", Flags::read | Flags::write);
}

EMT::Ph3::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String name,
	Logger::Level logLevel)
	: SynchronGeneratorIdeal(name, name, logLevel) { }

SimPowerComp<Real>::Ptr EMT::Ph3::SynchronGeneratorIdeal::clone(String name) {
	return SynchronGeneratorIdeal::make(name, mLogLevel);
}

void EMT::Ph3::SynchronGeneratorIdeal::initializeFromNodesAndTerminals(Real frequency) {
	mSubVoltageSource = EMT::Ph3::VoltageSource::make(mName + "_src", mLogLevel);
	mSubVoltageSource->connect({ SimNode::GND, node(0) });
	mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubVoltageSource->initialize(mFrequencies);
	mSubVoltageSource->initializeFromNodesAndTerminals(frequency);
	
	setAttributeRef("V_ref", mSubVoltageSource->attribute<MatrixComp>("V_ref"));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(mIntfVoltage),
		Logger::matrixToString(mIntfCurrent),
		Logger::phasorToString(initialSingleVoltage(0)));
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	mSubVoltageSource->mnaInitialize(omega, timeStep, leftVector);
	// since there is no additional behaviour, just use the tasks and right-vector from the voltage source
	setAttributeRef("right_vector", mSubVoltageSource->attribute("right_vector"));
	for (auto task : mSubVoltageSource->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubVoltageSource->mnaApplyRightSideVectorStamp(rightVector);
}

