/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#include <dpsim-models/DP/DP_Ph3_SeriesResistor.h>

using namespace CPS;

DP::Ph3::SeriesResistor::SeriesResistor(String uid, String name,
	Logger::Level logLevel)
	: MNASimPowerComp<Complex>(uid, name, false, true, logLevel), Base::Ph1::Resistor(mAttributes) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(2);
	**mIntfVoltage = MatrixComp::Zero(3,1);
	**mIntfCurrent = MatrixComp::Zero(3,1);
}

SimPowerComp<Complex>::Ptr DP::Ph3::SeriesResistor::clone(String name) {
	auto copy = SeriesResistor::make(name, mLogLevel);
	copy->setParameters(**mResistance);
	return copy;
}

void DP::Ph3::SeriesResistor::initializeFromNodesAndTerminals(Real frequency) {

	mTerminals[0]->setPhaseType(PhaseType::ABC);
	mTerminals[1]->setPhaseType(PhaseType::ABC);

	Matrix impedance = Matrix::Zero(3, 1);
	impedance <<
		**mResistance,
		**mResistance,
		**mResistance;
	(**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);

	Real voltMag = Math::abs((**mIntfVoltage)(0, 0));
	Real voltPhase = Math::phase((**mIntfVoltage)(0, 0));
	(**mIntfVoltage)(1, 0) = Complex(
		voltMag * cos(voltPhase - 2. / 3. * PI),
		voltMag * sin(voltPhase - 2. / 3. * PI));
	(**mIntfVoltage)(2, 0) = Complex(
		voltMag * cos(voltPhase + 2. / 3. * PI),
		voltMag * sin(voltPhase + 2. / 3. * PI));

	**mIntfCurrent = impedance.cwiseInverse().cwiseProduct(**mIntfVoltage);

	SPDLOG_LOGGER_DEBUG(mSLog, "\n--- Initialization from powerflow ---"
		"\nVoltage across amplitude and phase: \n{:s}"
		"\nCurrent amplitude and phase: \n{:s}"
		"\nTerminal 0 voltage amplitude and phase: \n{:s}"
		"\nTerminal 1 voltage amplitude and phase: \n{:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorMatrixToString(**mIntfVoltage),
		Logger::phasorMatrixToString(**mIntfCurrent),
		Logger::phasorMatrixToString(initialVoltage(0)),
		Logger::phasorMatrixToString(initialVoltage(1)));
}

void DP::Ph3::SeriesResistor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	**mRightVector = Matrix::Zero(0, 0);
}

void DP::Ph3::SeriesResistor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	Complex conductance = { (1./ **mResistance), 0 };

	//// Set diagonal entries
	//if (terminalNotGrounded(0))
	//	Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(0), conductance);
	//if (terminalNotGrounded(1))
	//	Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(1), conductance);
	//// Set off diagonal entries
	//if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
	//	Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(1), -conductance);
	//	Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(0), -conductance);
	//}
	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(0), conductance);

	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(1), conductance);

	}
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(1), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(0), -conductance);
	}

	if (terminalNotGrounded(0))
		SPDLOG_LOGGER_DEBUG(mSLog, "Add {} to {}, {}", conductance, matrixNodeIndex(0,0), matrixNodeIndex(0,0));
	if (terminalNotGrounded(1))
		SPDLOG_LOGGER_DEBUG(mSLog, "Add {} to {}, {}", conductance, matrixNodeIndex(1,0), matrixNodeIndex(1,0));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		SPDLOG_LOGGER_DEBUG(mSLog, "Add {} to {}, {}", -conductance, matrixNodeIndex(0,0), matrixNodeIndex(1,0));
		SPDLOG_LOGGER_DEBUG(mSLog, "Add {} to {}, {}", -conductance, matrixNodeIndex(1,0), matrixNodeIndex(0,0));
	}
}

void DP::Ph3::SeriesResistor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
		attributeDependencies.push_back(leftVector);
		modifiedAttributes.push_back(mIntfVoltage);
		modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::SeriesResistor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::SeriesResistor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	**mIntfVoltage = MatrixComp::Zero(3,1);
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1,0));
		(**mIntfVoltage)(1,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1,1));
		(**mIntfVoltage)(2,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1,2));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0,0));
		(**mIntfVoltage)(1,0) = (**mIntfVoltage)(1,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0,1));
		(**mIntfVoltage)(2,0) = (**mIntfVoltage)(2,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0,2));
	}

	SPDLOG_LOGGER_TRACE(mSLog, "Voltage A: {} < {}", std::abs((**mIntfVoltage)(0,0)), std::arg((**mIntfVoltage)(0,0)));
}

void DP::Ph3::SeriesResistor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = **mIntfVoltage / **mResistance;

	SPDLOG_LOGGER_TRACE(mSLog, "Current A: {} < {}", std::abs((**mIntfCurrent)(0,0)), std::arg((**mIntfCurrent)(0,0)));
}
