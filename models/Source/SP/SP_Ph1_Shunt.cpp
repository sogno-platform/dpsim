/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_Shunt.h>

using namespace CPS;

SP::Ph1::Shunt::Shunt(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {

	mSLog->info("Create {} of type {}", this->type(), name);
	setTerminalNumber(1);

	addAttribute<Real>("base_Voltage", &mBaseVoltage, Flags::read | Flags::write);
	addAttribute<Real>("G", &mConductance, Flags::write | Flags::read);
	addAttribute<Real>("B", &mSusceptance, Flags::write | Flags::read);
}


void SP::Ph1::Shunt::setParameters(Real conductance, Real susceptance){
	mConductance = conductance;
	mSusceptance = susceptance;
	mSLog->info("Conductance={} [S] Susceptance={} [Ohm] ", conductance, susceptance);
	mParametersSet = true;
}


// #### Powerflow section ####
void SP::Ph1::Shunt::setBaseVoltage(Real baseVoltage) {
    mBaseVoltage = baseVoltage;
}


void SP::Ph1::Shunt::calculatePerUnitParameters(Real baseApparentPower, Real baseOmega) {
	mSLog->info("#### Calculate Per Unit Parameters for {}", mName);
	mBaseApparentPower = baseApparentPower;
	mBaseOmega = baseOmega;
	mSLog->info("Base Power={} [VA]  Base Omega={} [1/s]", baseApparentPower, baseOmega);

	mBaseImpedance = (mBaseVoltage * mBaseVoltage) / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mSLog->info("Base Voltage={} [V]  Base Admittance={} [S]", mBaseVoltage, mBaseAdmittance);

	mConductancePerUnit = mConductance / mBaseAdmittance;
	mSusceptancePerUnit = mSusceptance / mBaseAdmittance;
	mSLog->info("Susceptance={} [pu] Conductance={} [pu]", mSusceptancePerUnit, mConductancePerUnit);
};


void SP::Ph1::Shunt::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
	int bus1 = this->matrixNodeIndex(0);
	Complex Y_element = Complex(mConductancePerUnit, mSusceptancePerUnit);

	if (std::isinf(Y_element.real()) || std::isinf(Y_element.imag())) {
		std::cout << "Y:" << Y_element << std::endl;
		std::stringstream ss;
		ss << "Shunt>>" << this->name() << ": infinite or nan values at node: " << bus1;
		throw std::invalid_argument(ss.str());
	}

	//set the circuit matrix values
	Y.coeffRef(bus1, bus1) += Y_element;
	mSLog->info("#### Y matrix stamping: {}", Y_element);

}
