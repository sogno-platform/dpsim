/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_Resistor.h>

using namespace CPS;


SP::Ph1::Resistor::Resistor(String uid, String name,
	Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("R", &mResistance, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr SP::Ph1::Resistor::clone(String name) {
	auto copy = Resistor::make(name, mLogLevel);
	copy->setParameters(mResistance);
	return copy;
}

void SP::Ph1::Resistor::setBaseVoltage(Real baseVoltage){
	mBaseVoltage = baseVoltage;
}

void SP::Ph1::Resistor::calculatePerUnitParameters(Real baseApparentPower){
	mSLog->info("#### Calculate Per Unit Parameters for {}", mName);
    mBaseApparentPower = baseApparentPower;
	mSLog->info("Base Power={} [VA]", baseApparentPower);

	mBaseImpedance = mBaseVoltage * mBaseVoltage / mBaseApparentPower;
	mBaseAdmittance = 1.0 / mBaseImpedance;
	mBaseCurrent = baseApparentPower / (mBaseVoltage*sqrt(3)); // I_base=(S_threephase/3)/(V_line_to_line/sqrt(3))
	mSLog->info("Base Voltage={} [V]  Base Impedance={} [Ohm]", mBaseVoltage, mBaseImpedance);

	mResistancePerUnit = mResistance / mBaseImpedance;
	mConductancePerUnit = 1. / mResistancePerUnit;
    mSLog->info("Resistance={} [pu]  Conductance={} [pu]", mResistancePerUnit, mConductancePerUnit);
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
	mSLog->info("#### Y matrix stamping: {}", Y_element);
}

void SP::Ph1::Resistor::initializeFromNodesAndTerminals(Real frequency) {

	mConductance = 1 / mResistance;
	mIntfVoltage(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent = mConductance*mIntfVoltage;

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void SP::Ph1::Resistor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)));
}

void SP::Ph1::Resistor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(0), mConductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(1), mConductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(1), -mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(0), -mConductance);
	}

	mSLog->info("-- Matrix Stamp ---");
	if (terminalNotGrounded(0))
		mSLog->info("Add {:e} to system at ({:d},{:d})",
			mConductance, matrixNodeIndex(0), matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:e} to system at ({:d},{:d})",
			mConductance, matrixNodeIndex(1), matrixNodeIndex(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:e} to system at ({:d},{:d})",
			-mConductance, matrixNodeIndex(0), matrixNodeIndex(1));
		mSLog->info("Add {:e} to system at ({:d},{:d})",
			-mConductance, matrixNodeIndex(1), matrixNodeIndex(0));
	}
}

void SP::Ph1::Resistor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mResistor.mnaUpdateVoltage(*mLeftVector);
	mResistor.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::Resistor::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage = MatrixComp::Zero(1, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	}

	//mLog.debug() << "Voltage A: " << std::abs(mIntfVoltage(0, 0))
	//	<< "<" << std::arg(mIntfVoltage(0, 0)) << std::endl;
}

void SP::Ph1::Resistor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mIntfVoltage / mResistance;

	//mLog.debug() << "Current A: " << std::abs(mIntfCurrent(0, 0))
	//	<< "<" << std::arg(mIntfCurrent(0, 0)) << std::endl;
}


void SP::Ph1::Resistor::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, Complex(mResistance, 0));
}
