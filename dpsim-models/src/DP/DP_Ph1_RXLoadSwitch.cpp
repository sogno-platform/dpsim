/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_RXLoadSwitch.h>

using namespace CPS;

DP::Ph1::RXLoadSwitch::RXLoadSwitch(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, true, true, logLevel) {
	setTerminalNumber(1);
	setVirtualNodeNumber(1);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);

	// Create sub components
	mSubRXLoad = std::make_shared<DP::Ph1::RXLoad>(**mName + "_rxload", mLogLevel);
	mSubSwitch = std::make_shared<DP::Ph1::Switch>(**mName + "_switch", mLogLevel);
	addMNASubComponent(mSubRXLoad, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	addMNASubComponent(mSubSwitch, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	// Set switch default values
	mSubSwitch->setParameters(1e9, 1e-9, true);
}

DP::Ph1::RXLoadSwitch::RXLoadSwitch(String name, Logger::Level logLevel)
	: RXLoadSwitch(name, name, logLevel) { }

Bool DP::Ph1::RXLoadSwitch::mnaIsClosed() { return mSubSwitch->isClosed(); }

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

	**mIntfVoltage = **mSubRXLoad->mIntfVoltage;
	**mIntfCurrent = **mSubRXLoad->mIntfCurrent;

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

void DP::Ph1::RXLoadSwitch::setParameters(Real activePower, Real reactivePower, Real nomVolt,
	Real openResistance, Real closedResistance, Bool closed) {
	mParametersSet = true;
	mSubRXLoad->setParameters(activePower, reactivePower, nomVolt);
	mSubSwitch->setParameters(openResistance, closedResistance, closed);
}

void DP::Ph1::RXLoadSwitch::setSwitchParameters(Real openResistance, Real closedResistance, Bool closed) {
	mSubSwitch->setParameters(openResistance, closedResistance, closed);
}

void DP::Ph1::RXLoadSwitch::mnaCompApplySwitchSystemMatrixStamp(Bool closed, SparseMatrixRow& systemMatrix, Int freqIdx) {
	mSubRXLoad->mnaApplySystemMatrixStamp(systemMatrix);
	mSubSwitch->mnaCompApplySwitchSystemMatrixStamp(closed, systemMatrix, freqIdx);
}

void DP::Ph1::RXLoadSwitch::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::RXLoadSwitch::mnaParentPreStep(Real time, Int timeStepCount) {
	updateSwitchState(time);
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::RXLoadSwitch::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::RXLoadSwitch::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	**mIntfVoltage = **mSubRXLoad->mIntfVoltage;
	**mIntfCurrent = **mSubRXLoad->mIntfCurrent;
}

void DP::Ph1::RXLoadSwitch::updateSwitchState(Real time) {

	if (time > mSwitchTimeOffset && mSubSwitch->isClosed()) {
		Real VRef = Math::abs(**mSubRXLoad->mNomVoltage);
		Real V = Math::abs((**mIntfVoltage)(0, 0));

		Real deltaV = Math::abs((V - VRef) / VRef);

		if (deltaV > 0.1) {
			mSubSwitch->open();
			SPDLOG_LOGGER_DEBUG(mSLog, "Opened Switch at {}", (float)time);
		}
	}
}
