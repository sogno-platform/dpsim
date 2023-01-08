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
	mSubVoltageSource->mVoltageRef->setReference(mVoltageRef);
	mSubVoltageSource->connect({ SimNode::GND, node(0) });
	mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubVoltageSource->initialize(mFrequencies);
	mSubVoltageSource->initializeFromNodesAndTerminals(frequency);
	mSubComponents.push_back(mSubVoltageSource);

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
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaInitialize(omega, timeStep, leftVector);
	}

	// collect tasks
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);
	}

	// add pre-step dependencies of component itself
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaPreStep(time, timeStepCount);
	}
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	}
	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaPostStep(time, timeStepCount, leftVector);
	}
	// post-step of component itself
	mnaUpdateCurrent(**leftVector);
	mnaUpdateVoltage(**leftVector);
}

void DP::Ph1::SynchronGeneratorIdeal::mnaUpdateCurrent(const Matrix& leftvector) {
	**mIntfCurrent = **mSubComponents[0]->mIntfCurrent;
}

void DP::Ph1::SynchronGeneratorIdeal::mnaUpdateVoltage(const Matrix& leftVector) {
	**mIntfVoltage = **mSubComponents[0]->mIntfVoltage;
}

void DP::Ph1::SynchronGeneratorIdeal::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);
	}
}

void DP::Ph1::SynchronGeneratorIdeal::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaApplyRightSideVectorStamp(rightVector);
	}
}
