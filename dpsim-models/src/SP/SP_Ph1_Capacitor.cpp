/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_Capacitor.h>

using namespace CPS;

SP::Ph1::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Complex>(uid, name, false, true, logLevel), Base::Ph1::Capacitor(mAttributes) {
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
	setTerminalNumber(2);
}

SimPowerComp<Complex>::Ptr SP::Ph1::Capacitor::clone(String name) {
	auto copy = Capacitor::make(name, mLogLevel);
	copy->setParameters(**mCapacitance);
	return copy;
}

void SP::Ph1::Capacitor::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;
	mSusceptance = Complex(0, omega * **mCapacitance);
	mImpedance = Complex(1, 0) / mSusceptance;
	mAdmittance = Complex(1, 0) / mImpedance;
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	**mIntfCurrent = mSusceptance * **mIntfVoltage;

	SPDLOG_LOGGER_INFO(mSLog, "\nCapacitance [F]: {:s}"
				"\nImpedance [Ohm]: {:s}"
				"\nAdmittance [S]: {:s}",
				Logger::realToString(**mCapacitance),
				Logger::complexToString(mImpedance),
				Logger::complexToString(mAdmittance));
	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
	mSLog->flush();
}

// #### MNA section ####

void SP::Ph1::Capacitor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)));
	mSLog->flush();
}

void SP::Ph1::Capacitor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {

	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mSusceptance);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mSusceptance);
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mSusceptance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mSusceptance);
	}

	SPDLOG_LOGGER_INFO(mSLog, "-- Matrix Stamp ---");
	if (terminalNotGrounded(0))
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			mSusceptance.real(), mSusceptance.imag(), matrixNodeIndex(0), matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			mSusceptance.real(), mSusceptance.imag(), matrixNodeIndex(1), matrixNodeIndex(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			-mSusceptance.real(), -mSusceptance.imag(), matrixNodeIndex(0), matrixNodeIndex(1));
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			-mSusceptance.real(), -mSusceptance.imag(), matrixNodeIndex(1), matrixNodeIndex(0));
	}
}

void SP::Ph1::Capacitor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::Capacitor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void SP::Ph1::Capacitor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	**mIntfVoltage = Matrix::Zero(1, 1);
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	}
}

void SP::Ph1::Capacitor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = mSusceptance * **mIntfVoltage;
}

// #### Powerflow section ####

void SP::Ph1::Capacitor::setBaseVoltage(Real baseVoltage){
	mBaseVoltage = baseVoltage;
}

void SP::Ph1::Capacitor::calculatePerUnitParameters(Real baseApparentPower){
	SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}", **mName);
    mBaseApparentPower = baseApparentPower;
	SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]", baseApparentPower);

	mBaseImpedance = mBaseVoltage * mBaseVoltage / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseCurrent = baseApparentPower / (mBaseVoltage*sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
	SPDLOG_LOGGER_INFO(mSLog, "Base Voltage={} [V]  Base Impedance={} [Ohm]", mBaseVoltage, mBaseImpedance);

	mImpedancePerUnit = mImpedance / mBaseImpedance;
	mAdmittancePerUnit = 1. / mImpedancePerUnit;
    SPDLOG_LOGGER_INFO(mSLog, "Impedance={} [pu]  Admittance={} [pu]", Logger::complexToString(mImpedancePerUnit), Logger::complexToString(mAdmittancePerUnit));
}

void SP::Ph1::Capacitor::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
	int bus1 = this->matrixNodeIndex(0);

	if (std::isinf(mAdmittancePerUnit.real()) || std::isinf(mAdmittancePerUnit.imag())) {
		std::cout << "Y:" << mAdmittancePerUnit << std::endl;
		std::stringstream ss;
		ss << "Capacitor >>" << this->name() << ": infinite or nan values at node: " << bus1;
		throw std::invalid_argument(ss.str());
	}

	//set the circuit matrix values
	Y.coeffRef(bus1, bus1) += mAdmittancePerUnit;
	SPDLOG_LOGGER_INFO(mSLog, "#### Y matrix stamping: {}", mAdmittancePerUnit);
}
