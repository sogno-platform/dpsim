/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_Shunt.h>

using namespace CPS;

SP::Ph1::Shunt::Shunt(String uid, String name, Logger::Level logLevel)
	: CompositePowerComp<Complex>(uid, name, false, true, logLevel),
	mConductance(mAttributes->create<Real>("G")),
	mSusceptance(mAttributes->create<Real>("B")),
	mConductancePerUnit(mAttributes->create<Real>("Gpu")),
	mSusceptancePerUnit(mAttributes->create<Real>("Bpu")) {

	SPDLOG_LOGGER_INFO(mSLog, "Create {} of type {}", this->type(), name);
	setTerminalNumber(1);
}

void SP::Ph1::Shunt::setParameters(Real conductance, Real susceptance) {
  **mConductance = conductance;
  **mSusceptance = susceptance;
  SPDLOG_LOGGER_INFO(mSLog, "Conductance={} [S] Susceptance={} [Ohm] ",
                     conductance, susceptance);
  mParametersSet = true;
}

// #### Powerflow section ####
void SP::Ph1::Shunt::setBaseVoltage(Real baseVoltage) {
  mBaseVoltage = baseVoltage;
}

void SP::Ph1::Shunt::calculatePerUnitParameters(Real baseApparentPower,
                                                Real baseOmega) {
  SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}",
                     **mName);
  SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]  Base Omega={} [1/s]",
                     baseApparentPower, baseOmega);

  auto baseImpedance = (mBaseVoltage * mBaseVoltage) / baseApparentPower;
  auto baseAdmittance = 1.0 / baseImpedance;
  SPDLOG_LOGGER_INFO(mSLog, "Base Voltage={} [V]  Base Admittance={} [S]",
                     mBaseVoltage, baseAdmittance);

  **mConductancePerUnit = **mConductance / baseAdmittance;
  **mSusceptancePerUnit = **mSusceptance / baseAdmittance;
  SPDLOG_LOGGER_INFO(mSLog, "Susceptance={} [pu] Conductance={} [pu]",
                     **mSusceptancePerUnit, **mConductancePerUnit);
};

void SP::Ph1::Shunt::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow &Y) {
  int bus1 = this->matrixNodeIndex(0);
  Complex Y_element = Complex(**mConductancePerUnit, **mSusceptancePerUnit);

  if (std::isinf(Y_element.real()) || std::isinf(Y_element.imag())) {
    std::cout << "Y:" << Y_element << std::endl;
    std::stringstream ss;
    ss << "Shunt>>" << this->name()
       << ": infinite or nan values at node: " << bus1;
    throw std::invalid_argument(ss.str());
  }

  //set the circuit matrix values
  Y.coeffRef(bus1, bus1) += Y_element;
  SPDLOG_LOGGER_INFO(mSLog, "#### Y matrix stamping: {}", Y_element);
}

/// MNA Section
void SP::Ph1::Shunt::initializeFromNodesAndTerminals(Real frequency) {

	// Static calculation
	Real omega = 2. * PI * frequency;
	Complex admittance = Complex(**mConductance, **mSusceptance);
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(0);
	(**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) * admittance;

	// Create series rl sub component
	if (**mConductance>0) {
		mSubResistor = std::make_shared<SP::Ph1::Resistor>(**mName + "_Res", mLogLevel);
		mSubResistor->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node()});
		mSubResistor->setParameters(1. / **mConductance);
		mSubResistor->initialize(mFrequencies);
		mSubResistor->initializeFromNodesAndTerminals(frequency);
		addMNASubComponent(mSubResistor, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
	}

	if (**mSusceptance>0) {
		mSubCapacitor = std::make_shared<SP::Ph1::Capacitor>(**mName + "_cap", mLogLevel);
		mSubCapacitor->setParameters(**mSusceptance / omega);
		mSubCapacitor->connect(SimNode::List{ SimNode::GND, mTerminals[0]->node()});
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

void SP::Ph1::Shunt::mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::Shunt::mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void SP::Ph1::Shunt::mnaCompUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void SP::Ph1::Shunt::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = 0;
	
	if (**mConductance>0)
		(**mIntfCurrent)(0, 0) += mSubResistor->intfCurrent()(0, 0); 
	if (**mSusceptance>0)
		(**mIntfCurrent)(0, 0) += mSubCapacitor->intfCurrent()(0, 0); 
}
