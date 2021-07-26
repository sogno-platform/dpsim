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
	
	// initialize subcomponent
	mSubVoltageSource->mnaInitialize(omega, timeStep, leftVector);

	// collect tasks
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	mSubVoltageSource->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	mSubVoltageSource->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(mRightVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	mSubVoltageSource->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	mSubVoltageSource->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	mnaUpdateCurrent(*leftVector);
	mnaUpdateVoltage(*leftVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaUpdateCurrent(const Matrix& leftvector) {
	mIntfCurrent = mSubVoltageSource->attribute<Matrix>("i_intf")->get();
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage = mSubVoltageSource->attribute<Matrix>("v_intf")->get();
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubVoltageSource->mnaApplyRightSideVectorStamp(rightVector);
}

