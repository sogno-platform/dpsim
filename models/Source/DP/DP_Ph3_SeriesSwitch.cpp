/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph3_SeriesSwitch.h>

using namespace CPS;

DP::Ph3::SeriesSwitch::SeriesSwitch(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(3,1);
	mIntfCurrent = MatrixComp::Zero(3,1);

	addAttribute<Real>("R_open", &mOpenResistance, Flags::read | Flags::write);
	addAttribute<Real>("R_closed", &mClosedResistance, Flags::read | Flags::write);
	addAttribute<Bool>("is_closed", &mIsClosed, Flags::read | Flags::write);
}

void DP::Ph3::SeriesSwitch::initializeFromNodesAndTerminals(Real frequency) {

	Real impedance = (mIsClosed) ? mClosedResistance : mOpenResistance;
	mIntfVoltage = initialVoltage(1) - initialVoltage(0);
	mIntfCurrent = mIntfVoltage / impedance;

	mSLog->info("\n--- Initialization from powerflow ---"
		"\nVoltage across phasor: \n{}"
		"\nCurrent phasor: \n{}"
		"\nTerminal 0 voltage phasor: \n{}"
		"\nTerminal 1 voltage phasor: \n{}",
		Logger::phasorToString(initialVoltage(0)(0,0)),
		Logger::phasorToString(initialVoltage(1)(0,0)),
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)));
}

void DP::Ph3::SeriesSwitch::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph3::SeriesSwitch::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Complex conductance = (mIsClosed)
		? Complex( 1./mClosedResistance, 0 )
		: Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(1), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(0), -conductance);
	}

	if (terminalNotGrounded(0))
		mSLog->info("Add {} to {}, {}", conductance, matrixNodeIndices(0)[0], matrixNodeIndices(0)[0]);
	if (terminalNotGrounded(1))
		mSLog->info("Add {} to {}, {}", conductance, matrixNodeIndices(1)[0], matrixNodeIndices(1)[0]);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {} to {}, {}", -conductance, matrixNodeIndices(0)[0], matrixNodeIndices(1)[0]);
		mSLog->info("Add {} to {}, {}", -conductance, matrixNodeIndices(1)[0], matrixNodeIndices(0)[0]);
	}
}

void DP::Ph3::SeriesSwitch::mnaApplySwitchSystemMatrixStamp(Bool closed, Matrix& systemMatrix, Int freqIdx) {
	Complex conductance = (closed)
		? Complex( 1./mClosedResistance, 0 )
		: Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(0), matrixNodeIndices(1), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndices(1), matrixNodeIndices(0), -conductance);
	}

	if (terminalNotGrounded(0))
		mSLog->info("Add {} to {}, {}", conductance, matrixNodeIndices(0)[0], matrixNodeIndices(0)[0]);
	if (terminalNotGrounded(1))
		mSLog->info("Add {} to {}, {}", conductance, matrixNodeIndices(1)[0], matrixNodeIndices(1)[0]);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {} to {}, {}", -conductance, matrixNodeIndices(0)[0], matrixNodeIndices(1)[0]);
		mSLog->info("Add {} to {}, {}", -conductance, matrixNodeIndices(1)[0], matrixNodeIndices(0)[0]);
	}
}

void DP::Ph3::SeriesSwitch::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSwitch.mnaUpdateVoltage(*mLeftVector);
	mSwitch.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph3::SeriesSwitch::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage = MatrixComp::Zero(3,1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1,0));
		mIntfVoltage(1,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1,1));
		mIntfVoltage(2,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1,2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0,0));
		mIntfVoltage(1,0) = mIntfVoltage(1,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0,1));
		mIntfVoltage(2,0) = mIntfVoltage(2,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0,2));
	}

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {} < {}", std::abs(mIntfVoltage(0,0)), std::arg(mIntfVoltage(0,0)));
}

void DP::Ph3::SeriesSwitch::mnaUpdateCurrent(const Matrix& leftVector) {
	Real impedance = (mIsClosed)? mClosedResistance : mOpenResistance;
	mIntfCurrent = mIntfVoltage / impedance;

	SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}", std::abs(mIntfCurrent(0,0)), std::arg(mIntfCurrent(0,0)));
}
