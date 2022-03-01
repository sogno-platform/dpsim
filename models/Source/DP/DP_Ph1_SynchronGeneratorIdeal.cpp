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
	: SimPowerComp<Complex>(uid, name, logLevel),
	mVoltageRef(Attribute<Complex>::createDynamic("V_ref", mAttributes)) {
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
	mSubComponents.push_back(mSubVoltageSource);
	mSubComponents[0]->connect({ SimNode::GND, node(0) });
	mSubComponents[0]->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubComponents[0]->initialize(mFrequencies);
	mSubComponents[0]->initializeFromNodesAndTerminals(frequency);

	mVoltageRef->setReference(mSubComponents[0]->attribute<Complex>("V_ref"));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)));
}

void DP::Ph1::SynchronGeneratorIdeal::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	// initialize subcomponent
	std::dynamic_pointer_cast<MNAInterface>(mSubComponents[0])->mnaInitialize(omega, timeStep, leftVector);

	// collect tasks
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	std::dynamic_pointer_cast<MNAInterface>(mSubComponents[0])->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void DP::Ph1::SynchronGeneratorIdeal::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	std::dynamic_pointer_cast<MNAInterface>(mSubComponents[0])->mnaPreStep(time, timeStepCount);
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	std::dynamic_pointer_cast<MNAInterface>(mSubComponents[0])->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void DP::Ph1::SynchronGeneratorIdeal::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	std::dynamic_pointer_cast<MNAInterface>(mSubComponents[0])->mnaPostStep(time, timeStepCount, leftVector);
	// post-step of component itself
	mnaUpdateCurrent(**leftVector);
	mnaUpdateVoltage(**leftVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaUpdateCurrent(const Matrix& leftvector) {
	**mIntfCurrent = mSubComponents[0]->attribute<MatrixComp>("i_intf")->get();
}

void DP::Ph1::SynchronGeneratorIdeal::mnaUpdateVoltage(const Matrix& leftVector) {
	**mIntfVoltage = mSubComponents[0]->attribute<MatrixComp>("v_intf")->get();
}

void DP::Ph1::SynchronGeneratorIdeal::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	std::dynamic_pointer_cast<MNAInterface>(mSubComponents[0])->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	std::dynamic_pointer_cast<MNAInterface>(mSubComponents[0])->mnaApplyRightSideVectorStamp(rightVector);
}
