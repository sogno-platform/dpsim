/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_PQLoadCS.h>

using namespace CPS;

DP::Ph1::PQLoadCS::PQLoadCS(String uid, String name,
	Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	mActivePower(mAttributes->create<Real>("P", 0)),
	mReactivePower(mAttributes->create<Real>("Q", 0)),
	mNomVoltage(mAttributes->create<Real>("V_nom")) {
	setTerminalNumber(1);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
}

DP::Ph1::PQLoadCS::PQLoadCS(String uid, String name,
	Real activePower, Real reactivePower, Real nomVolt,
	Logger::Level logLevel)
	: PQLoadCS(uid, name, logLevel) {

	setParameters(activePower, reactivePower, nomVolt);
}

DP::Ph1::PQLoadCS::PQLoadCS(String name, Logger::Level logLevel)
	: PQLoadCS(name, name, logLevel) {
}

DP::Ph1::PQLoadCS::PQLoadCS(String name,
	Real activePower, Real reactivePower, Real nomVolt,
	Logger::Level logLevel)
	: PQLoadCS(name, name, activePower, reactivePower, nomVolt, logLevel) {
}

void DP::Ph1::PQLoadCS::setParameters(Real activePower, Real reactivePower, Real nomVolt) {
	**mActivePower = activePower;
	**mReactivePower = reactivePower;
	**mNomVoltage = nomVolt;
	mParametersSet = true;
}

///DEPRECATED: Delete method
SimPowerComp<Complex>::Ptr DP::Ph1::PQLoadCS::clone(String name) {
	auto copy = PQLoadCS::make(name, mLogLevel);
	copy->setParameters(**mActivePower, **mReactivePower, **mNomVoltage);
	return copy;
}

void DP::Ph1::PQLoadCS::initializeFromNodesAndTerminals(Real frequency) {
	// Get power from Terminals if it was not set previously.
	if (**mActivePower == 0 && **mReactivePower == 0 && !mParametersSet) {
		**mActivePower = mTerminals[0]->singleActivePower();
		**mReactivePower = mTerminals[0]->singleReactivePower();
		**mNomVoltage = std::abs(mTerminals[0]->initialSingleVoltage());
	}
	Complex power = Complex(**mActivePower, **mReactivePower);

	Complex current;
	if (**mNomVoltage != 0)
		current = std::conj(power / **mNomVoltage);
	else
		current = 0;

	mSubCurrentSource = std::make_shared<DP::Ph1::CurrentSource>(**mName + "_cs", mLogLevel);
	mSubCurrentSource->setParameters(current);
	// A positive power should result in a positive current to ground.
	mSubCurrentSource->connect({ mTerminals[0]->node(), SimNode::GND });
	mSubCurrentSource->initializeFromNodesAndTerminals(frequency);
	addMNASubComponent(mSubCurrentSource, MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	updateIntfValues();

	SPDLOG_LOGGER_DEBUG(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nCurrent set point: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(current));
}

void DP::Ph1::PQLoadCS::updateSetPoint() {
	// Calculate new current set point.
	Complex power = { **mActivePower, **mReactivePower};
	Complex current = power / **mNomVoltage;
	//Complex current = power / (**mIntfVoltage)(0,0);

	**mSubCurrentSource->mCurrentRef = std::conj(current);
	SPDLOG_LOGGER_DEBUG(mSLog,
		"\n--- update set points ---"
		"\npower: {:s}"
		"\nCurrent: {:s}",
		Logger::phasorToString(power),
		Logger::phasorToString(std::conj(current)));
}

void DP::Ph1::PQLoadCS::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mActivePower);
	attributeDependencies.push_back(mReactivePower);
	attributeDependencies.push_back(mNomVoltage);
}

void DP::Ph1::PQLoadCS::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	modifiedAttributes.push_back(mIntfCurrent);
	modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph1::PQLoadCS::mnaParentPreStep(Real time, Int timeStepCount) {
	updateSetPoint();
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::PQLoadCS::updateIntfValues() {
	**mIntfCurrent = mSubCurrentSource->intfCurrent();
	**mIntfVoltage = mSubCurrentSource->intfVoltage();
}

void DP::Ph1::PQLoadCS::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	updateIntfValues();
}

