/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_Shunt.h>

using namespace CPS;

EMT::Ph3::Shunt::Shunt(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Real>(uid, name, false, true, logLevel) {

	mConductance = Matrix::Zero(3,3);
	mSusceptance = Matrix::Zero(3,3);
	SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", this->type(), name);
	setTerminalNumber(1);
}


void EMT::Ph3::Shunt::setParameters(Real conductance, Real susceptance) {
	this->setParameters(Math::singlePhaseParameterToThreePhase(conductance), 
						Math::singlePhaseParameterToThreePhase(susceptance));
}

void EMT::Ph3::Shunt::setParameters(Matrix conductance, Matrix susceptance){
	mConductance = conductance;
	mSusceptance = susceptance;
	SPDLOG_LOGGER_INFO(mSLog, "Conductance={} [S] Susceptance={} [Ohm] ", 
					   Logger::matrixToString(mConductance), 
					   Logger::matrixToString(mSusceptance));
	mParametersSet = true;
}

/// MNA Section
void EMT::Ph3::Shunt::initializeFromNodesAndTerminals(Real frequency) {

	// Static calculation
	Real omega = 2. * PI * frequency;
	MatrixComp admittance = Matrix::Zero(3,3);
	admittance << Complex(mConductance(0,0), mSusceptance(0,0)), 0, 0,
				  0, Complex(mConductance(1,1), mSusceptance(1,1)), 0,
				  0, 0, Complex(mConductance(2,2), mSusceptance(2,2));
	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * mTerminals[0]->initialSingleVoltage();
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	**mIntfVoltage = vInitABC.real();
	**mIntfCurrent =  (admittance * vInitABC).real();

	// Create series rl sub component
	if (mConductance(0,0)>0) {
		mSubResistor = std::make_shared<EMT::Ph3::Resistor>(**mName + "_Res", mLogLevel);
		mSubResistor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubResistor->setParameters(mConductance.inverse());
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	}

	if (mSusceptance(0,0)>0) {
		mSubCapacitor = std::make_shared<EMT::Ph3::Capacitor>(**mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(mSusceptance * (1/omega));
		mSubCapacitor->connect({ SimNode::GND, mTerminals[0]->node() });
		mSubCapacitor->initialize(mFrequencies);
		mSubCapacitor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubCapacitor, MNA_SUBCOMP_TASK_ORDER::NO_TASK, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
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

void EMT::Ph3::Shunt::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::Shunt::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void EMT::Ph3::Shunt::mnaCompUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	(**mIntfVoltage)(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	(**mIntfVoltage)(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}

void EMT::Ph3::Shunt::mnaCompUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = Matrix::Zero(3, 1);
	
	if (mConductance(0,0)>0)
		**mIntfCurrent += mSubResistor->intfCurrent(); 
	if (mSusceptance(0,0)>0)
		**mIntfCurrent += mSubCapacitor->intfCurrent(); 
}
