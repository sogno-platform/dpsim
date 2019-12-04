/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/DP/DP_Ph3_SeriesSwitch.h>

using namespace CPS;

DP::Ph3::SeriesSwitch::SeriesSwitch(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(3,1);
	mIntfCurrent = MatrixComp::Zero(3,1);

	addAttribute<Real>("R_open", &mOpenResistance, Flags::read | Flags::write);
	addAttribute<Real>("R_closed", &mClosedResistance, Flags::read | Flags::write);
	addAttribute<Bool>("is_closed", &mIsClosed, Flags::read | Flags::write);
}

void DP::Ph3::SeriesSwitch::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

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
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph3::SeriesSwitch::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Complex conductance = (mIsClosed)
		? Complex( 1./mClosedResistance, 0 )
		: Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNodes(0), simNodes(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNodes(1), simNodes(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNodes(0), simNodes(1), -conductance);
		Math::addToMatrixElement(systemMatrix, simNodes(1), simNodes(0), -conductance);
	}

	if (terminalNotGrounded(0))
		mSLog->info("Add {} to {}, {}", conductance, simNodes(0)[0], simNodes(0)[0]);
	if (terminalNotGrounded(1))
		mSLog->info("Add {} to {}, {}", conductance, simNodes(1)[0], simNodes(1)[0]);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {} to {}, {}", -conductance, simNodes(0)[0], simNodes(1)[0]);
		mSLog->info("Add {} to {}, {}", -conductance, simNodes(1)[0], simNodes(0)[0]);
	}
}

void DP::Ph3::SeriesSwitch::mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed) {
	Complex conductance = (closed)
		? Complex( 1./mClosedResistance, 0 )
		: Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNodes(0), simNodes(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNodes(1), simNodes(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNodes(0), simNodes(1), -conductance);
		Math::addToMatrixElement(systemMatrix, simNodes(1), simNodes(0), -conductance);
	}

	if (terminalNotGrounded(0))
		mSLog->info("Add {} to {}, {}", conductance, simNodes(0)[0], simNodes(0)[0]);
	if (terminalNotGrounded(1))
		mSLog->info("Add {} to {}, {}", conductance, simNodes(1)[0], simNodes(1)[0]);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {} to {}, {}", -conductance, simNodes(0)[0], simNodes(1)[0]);
		mSLog->info("Add {} to {}, {}", -conductance, simNodes(1)[0], simNodes(0)[0]);
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
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(1,0));
		mIntfVoltage(1,0) = Math::complexFromVectorElement(leftVector, simNode(1,1));
		mIntfVoltage(2,0) = Math::complexFromVectorElement(leftVector, simNode(1,2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, simNode(0,0));
		mIntfVoltage(1,0) = mIntfVoltage(1,0) - Math::complexFromVectorElement(leftVector, simNode(0,1));
		mIntfVoltage(2,0) = mIntfVoltage(2,0) - Math::complexFromVectorElement(leftVector, simNode(0,2));
	}

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {} < {}", std::abs(mIntfVoltage(0,0)), std::arg(mIntfVoltage(0,0)));
}

void DP::Ph3::SeriesSwitch::mnaUpdateCurrent(const Matrix& leftVector) {
	Real impedance = (mIsClosed)? mClosedResistance : mOpenResistance;
	mIntfCurrent = mIntfVoltage / impedance;

	SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}", std::abs(mIntfCurrent(0,0)), std::arg(mIntfCurrent(0,0)));
}
