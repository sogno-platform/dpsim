/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/SP/SP_Ph1_Resistor.h>

using namespace CPS;


SP::Ph1::Resistor::Resistor(String uid, String name,
	Logger::Level logLevel)
	: MNASimPowerComp<Complex>(uid, name, false, true, logLevel), Base::Ph1::Resistor(mAttributes) {
	setTerminalNumber(2);
	**mIntfVoltage = MatrixComp::Zero(1, 1);
	**mIntfCurrent = MatrixComp::Zero(1, 1);
}

SimPowerComp<Complex>::Ptr SP::Ph1::Resistor::clone(String name) {
	auto copy = Resistor::make(name, mLogLevel);
	copy->setParameters(**mResistance);
	return copy;
}

void SP::Ph1::Resistor::initializeFromNodesAndTerminals(Real frequency) {

	(**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	**mIntfCurrent = (1 / **mResistance) * **mIntfVoltage;

	SPDLOG_LOGGER_INFO(mSLog, "\nResistance [Ohm]: {:s}",
				Logger::realToString(**mResistance));
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

// #### Powerflow section ####

void SP::Ph1::Resistor::setBaseVoltage(Real baseVoltage){
	mBaseVoltage = baseVoltage;
}

void SP::Ph1::Resistor::calculatePerUnitParameters(Real baseApparentPower){
	SPDLOG_LOGGER_INFO(mSLog, "#### Calculate Per Unit Parameters for {}", **mName);
    mBaseApparentPower = baseApparentPower;
	SPDLOG_LOGGER_INFO(mSLog, "Base Power={} [VA]", baseApparentPower);

	mBaseImpedance = mBaseVoltage * mBaseVoltage / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseCurrent = baseApparentPower / (mBaseVoltage*sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
	SPDLOG_LOGGER_INFO(mSLog, "Base Voltage={} [V]  Base Impedance={} [Ohm]", mBaseVoltage, mBaseImpedance);

	mResistancePerUnit = **mResistance / mBaseImpedance;
	mConductancePerUnit = 1. / mResistancePerUnit;
    SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [pu]  Conductance={} [pu]", mResistancePerUnit, mConductancePerUnit);
}

void SP::Ph1::Resistor::pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) {
		int bus1 = this->matrixNodeIndex(0);
		Complex Y_element = Complex(mConductancePerUnit, 0);

	if (std::isinf(Y_element.real()) || std::isinf(Y_element.imag())) {
		std::cout << "Y:" << Y_element << std::endl;
		std::stringstream ss;
		ss << "Resistor >>" << this->name() << ": infinite or nan values at node: " << bus1;
		throw std::invalid_argument(ss.str());
	}

	//set the circuit matrix values
	Y.coeffRef(bus1, bus1) += Y_element;
	SPDLOG_LOGGER_INFO(mSLog, "#### Y matrix stamping: {}", Y_element);
}

// #### MNA section ####

void SP::Ph1::Resistor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	**mRightVector = Matrix::Zero(0, 0);

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString((**mIntfCurrent)(0, 0)));
	mSLog->flush();
}

void SP::Ph1::Resistor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	Complex conductance = Complex(1. / **mResistance, 0);

	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		// Set diagonal entries
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), conductance, mNumFreqs, freq);
		if (terminalNotGrounded(1))
		// Set off diagonal entries
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), conductance, mNumFreqs, freq);
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -conductance, mNumFreqs, freq);
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -conductance, mNumFreqs, freq);
		}

		SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freq);
		if (terminalNotGrounded(0))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), matrixNodeIndex(1), matrixNodeIndex(1));
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), matrixNodeIndex(0), matrixNodeIndex(1));
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), matrixNodeIndex(1), matrixNodeIndex(0));
		}
	}
	mSLog->flush();
}

void SP::Ph1::Resistor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void SP::Ph1::Resistor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void SP::Ph1::Resistor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfVoltage)(0,freq) = 0;
		if (terminalNotGrounded(1))
			(**mIntfVoltage)(0,freq) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1), mNumFreqs, freq);
		if (terminalNotGrounded(0))
			(**mIntfVoltage)(0,freq) = (**mIntfVoltage)(0,freq) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString((**mIntfVoltage)(0,freq)));
	}
}

void SP::Ph1::Resistor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfCurrent)(0,freq) = (**mIntfVoltage)(0,freq) / **mResistance;
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString((**mIntfCurrent)(0,freq)));
	}
}


void SP::Ph1::Resistor::mnaTearApplyMatrixStamp(SparseMatrixRow& tearMatrix) {
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, Complex(**mResistance, 0));
}
