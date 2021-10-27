/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_RXLoadSwitch.h>

using namespace CPS;

DP::Ph1::RXLoadSwitch::RXLoadSwitch(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setTerminalNumber(1);
	setVirtualNodeNumber(1);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	// Create sub components
	mSubRXLoad = std::make_shared<DP::Ph1::RXLoad>(mName + "_rxload", mLogLevel);
	mSubSwitch = std::make_shared<DP::Ph1::Switch>(mName + "_switch", mLogLevel);
	mSubComponents.push_back(mSubRXLoad);
	mSubComponents.push_back(mSubSwitch);
	// Set switch default values
	mSubSwitch->setParameters(1e9, 1e-9, true);
}

DP::Ph1::RXLoadSwitch::RXLoadSwitch(String name, Logger::Level logLevel)
	: RXLoadSwitch(name, name, logLevel) { }

void DP::Ph1::RXLoadSwitch::initializeFromNodesAndTerminals(Real frequency) {

	if(!mParametersSet) {
		// use powerflow results
		mSubRXLoad->setParameters(
			mTerminals[0]->singleActivePower(),
			mTerminals[0]->singleReactivePower(),
			std::abs(mTerminals[0]->initialSingleVoltage()));
	}

	mSubRXLoad->connect({ virtualNode(0) });
	mSubRXLoad->initialize(mFrequencies);
	mSubRXLoad->initializeFromNodesAndTerminals(frequency);

	mSubSwitch->connect({ node(0) });
	mSubSwitch->initialize(mFrequencies);
	mSubSwitch->initializeFromNodesAndTerminals(frequency);

	mIntfVoltage = mSubRXLoad->attributeMatrixComp("v_intf")->get();
	mIntfCurrent = mSubRXLoad->attributeMatrixComp("i_intf")->get();

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

void DP::Ph1::RXLoadSwitch::setParameters(Real activePower, Real reactivePower, Real nomVolt,
	Real openResistance, Real closedResistance, Bool closed) {
	mParametersSet = true;
	mSubRXLoad->setParameters(activePower, reactivePower, nomVolt);
	mSubSwitch->setParameters(openResistance, closedResistance, closed);
}

void DP::Ph1::RXLoadSwitch::setSwitchParameters(Real openResistance, Real closedResistance, Bool closed) {
	mSubSwitch->setParameters(openResistance, closedResistance, closed);
}

void DP::Ph1::RXLoadSwitch::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mSubRXLoad->mnaInitialize(omega, timeStep, leftVector);
	mSubSwitch->mnaInitialize(omega, timeStep, leftVector);
	// get sub component right vector
	mRightVectorStamps.push_back(&mSubRXLoad->attribute<Matrix>("right_vector")->get());

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph1::RXLoadSwitch::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	rightVector.setZero();
	for (auto stamp : mRightVectorStamps)
		rightVector += *stamp;
}

void DP::Ph1::RXLoadSwitch::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubRXLoad->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSwitch->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::RXLoadSwitch::mnaApplySwitchSystemMatrixStamp(Bool closed, Matrix& systemMatrix, Int freqIdx) {
	mSubRXLoad->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSwitch->mnaApplySwitchSystemMatrixStamp(closed, systemMatrix, freqIdx);
}

void DP::Ph1::RXLoadSwitch::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// add pre-step dependencies of subcomponents
	this->mSubRXLoad->mnaAddPreStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes);

	// add pre-step dependencies of component
	prevStepDependencies.push_back(attribute("i_intf"));
	prevStepDependencies.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void DP::Ph1::RXLoadSwitch::mnaPreStep(Real time, Int timeStepCount) {
	mSubRXLoad->mnaPreStep(time, timeStepCount);

	// pre-step of component itself
	updateSwitchState(time);
	mnaApplyRightSideVectorStamp(mRightVector);
}

void DP::Ph1::RXLoadSwitch::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	// add post-step dependencies of subcomponents
	mSubRXLoad->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);
	mSubSwitch->mnaAddPostStepDependencies(prevStepDependencies, attributeDependencies, modifiedAttributes, leftVector);

	// add post-step dependencies of component itself
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void DP::Ph1::RXLoadSwitch::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	// post-step of subcomponents
	this->mSubRXLoad->mnaPostStep(time, timeStepCount, leftVector);
	this->mSubSwitch->mnaPostStep(time, timeStepCount, leftVector);

	mIntfVoltage = mSubRXLoad->attributeMatrixComp("v_intf")->get();
	mIntfCurrent = mSubRXLoad->attributeMatrixComp("i_intf")->get();
}

void DP::Ph1::RXLoadSwitch::updateSwitchState(Real time) {

	if (time > mSwitchTimeOffset && mSubSwitch->isClosed()) {
		Real VRef = Math::abs(mSubRXLoad->attributeComplex("V_nom")->get());
		Real V = Math::abs(mIntfVoltage(0, 0));

		Real deltaV = Math::abs((V - VRef) / VRef);

		if (deltaV > 0.1) {
			mSubSwitch->open();
			mSLog->info("Opened Switch at {}", (float)time);
		}
	}
}
