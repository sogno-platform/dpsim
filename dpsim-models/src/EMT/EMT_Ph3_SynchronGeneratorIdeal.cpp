/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorIdeal.h>

using namespace CPS;


EMT::Ph3::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String uid, String name, Logger::Level logLevel, CPS::GeneratorType sourceType)
	: SimPowerComp<Real>(uid, name, logLevel),
	mRefVoltage(Attribute<MatrixComp>::createDynamic("V_ref", mAttributes)) {
	mPhaseType = PhaseType::ABC;
	mSourceType = sourceType;

	if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
		setVirtualNodeNumber(1);
	else
		setVirtualNodeNumber(0);

	setTerminalNumber(1);
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
}

EMT::Ph3::SynchronGeneratorIdeal::SynchronGeneratorIdeal(String name,
	Logger::Level logLevel)
	: SynchronGeneratorIdeal(name, name, logLevel) { }

SimPowerComp<Real>::Ptr EMT::Ph3::SynchronGeneratorIdeal::clone(String name) {
	return SynchronGeneratorIdeal::make(name, mLogLevel);
}

void EMT::Ph3::SynchronGeneratorIdeal::initializeFromNodesAndTerminals(Real frequency) {

	if (mSourceType == CPS::GeneratorType::IdealVoltageSource) {
		mSubVoltageSource = EMT::Ph3::VoltageSource::make(**mName + "_vs", mLogLevel);
		mSubComponents.push_back(mSubVoltageSource);
	} else {
		mSubCurrentSource = EMT::Ph3::CurrentSource::make(**mName + "_cs", mLogLevel);
		mSubComponents.push_back(mSubCurrentSource);
	}

	mSubComponents[0]->connect({ SimNode::GND, node(0) });

	if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
		mSubComponents[0]->setVirtualNodeAt(mVirtualNodes[0], 0);

	if (mSourceType == CPS::GeneratorType::IdealCurrentSource)
		mSubComponents[0]->setTerminalAt(terminal(0), 1);

	mSubComponents[0]->initialize(mFrequencies);
	mSubComponents[0]->initializeFromNodesAndTerminals(frequency);

	if (mSourceType == CPS::GeneratorType::IdealVoltageSource)
		mRefVoltage->setReference(mSubComponents[0]->attributeTyped<MatrixComp>("V_ref"));

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 0 power: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::complexToString(terminal(0)->singlePower()));
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
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

void EMT::Ph3::SynchronGeneratorIdeal::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
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

void EMT::Ph3::SynchronGeneratorIdeal::mnaPreStep(Real time, Int timeStepCount) {
	// pre-step of subcomponents
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaPreStep(time, timeStepCount);
	}
	// pre-step of component itself
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
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

void EMT::Ph3::SynchronGeneratorIdeal::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaPostStep(time, timeStepCount, leftVector);
	}
	// post-step of component itself
	mnaUpdateCurrent(**leftVector);
	mnaUpdateVoltage(**leftVector);
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaUpdateCurrent(const Matrix& leftvector) {
	**mIntfCurrent = **mSubComponents[0]->mIntfCurrent;
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaUpdateVoltage(const Matrix& leftVector) {
	**mIntfVoltage = **mSubComponents[0]->mIntfVoltage;
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);
	}
}

void EMT::Ph3::SynchronGeneratorIdeal::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	for (auto subComp : mSubComponents) {
		if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subComp))
			mnasubcomp->mnaApplyRightSideVectorStamp(rightVector);
	}
}
