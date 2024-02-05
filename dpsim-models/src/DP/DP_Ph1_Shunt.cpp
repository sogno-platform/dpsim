/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_Shunt.h>

using namespace CPS;

DP::Ph1::Shunt::Shunt(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, false, true, logLevel),
	mConductance(mAttributes->create<Real>("G")),
	mSusceptance(mAttributes->create<Real>("B")) {

	SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", this->type(), name);
	setTerminalNumber(1);
}


void DP::Ph1::Shunt::setParameters(Real conductance, Real susceptance){
	**mConductance = conductance;
	**mSusceptance = susceptance;
	SPDLOG_LOGGER_INFO(mSLog, "Conductance={} [S] Susceptance={} [Ohm] ", conductance, susceptance);
	mParametersSet = true;
}

/// MNA Section
void DP::Ph1::Shunt::initializeFromNodesAndTerminals(Real frequency) {

	// Static calculation
	Real omega = 2. * PI * frequency;
	Complex admittance = Complex(**mConductance, **mSusceptance);
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) * admittance;

	// Create series rl sub component
	if (**mConductance>0) {
		mSubResistor = std::make_shared<DP::Ph1::Resistor>(**mName + "_Res", mLogLevel);
		mSubResistor->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node()});
		mSubResistor->setParameters(1. / **mConductance);
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	if (**mSusceptance>0) {
		mSubCapacitor = std::make_shared<DP::Ph1::Capacitor>(**mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(**mSusceptance / omega);
		mSubCapacitor->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node()});
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubCapacitor, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
	}

	SPDLOG_LOGGER_INFO(mSLog, 
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)));
}

void DP::Ph1::Shunt::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::Shunt::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void DP::Ph1::Shunt::mnaCompUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::Shunt::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = 0;
	
	if (**mConductance>0)
		(**mIntfCurrent)(0, 0) += mSubResistor->intfCurrent()(0, 0); 
	if (**mSusceptance>0)
		(**mIntfCurrent)(0, 0) += mSubCapacitor->intfCurrent()(0, 0); 
}
