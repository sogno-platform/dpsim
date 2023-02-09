/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_RXLoad.h>

using namespace CPS;

DP::Ph1::RXLoad::RXLoad(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, true, true, logLevel),
	mActivePower(mAttributes->create<Real>("P")),
	mReactivePower(mAttributes->create<Real>("Q")),
	mNomVoltage(mAttributes->create<Real>("V_nom")) {
	setTerminalNumber(1);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
}

DP::Ph1::RXLoad::RXLoad(String name, Logger::Level logLevel)
	: RXLoad(name, name, logLevel) {
}

void DP::Ph1::RXLoad::initializeFromNodesAndTerminals(Real frequency) {

	if(!mParametersSet){
		setParameters(
			mTerminals[0]->singleActivePower(),
			mTerminals[0]->singleReactivePower());
	}
	if (**mNomVoltage==0) {
		**mNomVoltage = std::abs(initialSingleVoltage(0));
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
	}

	if (**mActivePower != 0) {
		mResistance = std::pow(**mNomVoltage, 2) / **mActivePower;
		mSubResistor = std::make_shared<DP::Ph1::Resistor>(**mName + "_res", mLogLevel);
		mSubResistor->setParameters(mResistance);
		mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	}
	else {
		mResistance = 0;
	}

	if (**mReactivePower != 0)
		mReactance = std::pow(**mNomVoltage, 2) / **mReactivePower;
	else
		mReactance = 0;

	if (mReactance > 0) {
		mInductance = mReactance / (2.*PI*frequency);
		mSubInductor = std::make_shared<DP::Ph1::Inductor>(**mName + "_ind", mLogLevel);
		mSubInductor->setParameters(mInductance);
		mSubInductor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubInductor->initialize(mFrequencies);
		mSubInductor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}
	else if (mReactance < 0) {
		mCapacitance = -1. / (2.*PI*frequency) / mReactance;
		mSubCapacitor = std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mCapacitance);
		mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubCapacitor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	(**mIntfVoltage)(0, 0) = mTerminals[0]->initialSingleVoltage();
	(**mIntfCurrent)(0, 0) = std::conj(Complex(**mActivePower, **mReactivePower) / (**mIntfVoltage)(0, 0));

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nResistance: {:f}"
		"\nReactance: {:f}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		mResistance,
		mReactance);
}

void DP::Ph1::RXLoad::setParameters(Real activePower, Real reactivePower, Real volt) {
	mParametersSet = true;
	**mActivePower = activePower;
	**mReactivePower = reactivePower;
	**mNomVoltage = volt;

	SPDLOG_LOGGER_INFO(mSLog, "Active Power={} [W] Reactive Power={} [VAr]", **mActivePower, **mReactivePower);
	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage={} [V]", **mNomVoltage);
}

void DP::Ph1::RXLoad::mnaCompUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::RXLoad::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = 0;

	for (auto subComp : mSubComponents) {
		(**mIntfCurrent)(0, 0) += subComp->intfCurrent()(0, 0);
	}
}

void DP::Ph1::RXLoad::mnaParentAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::RXLoad::mnaParentPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::RXLoad::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::RXLoad::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}
