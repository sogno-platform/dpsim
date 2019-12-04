/**
 * @file
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

#include <cps/EMT/EMT_Ph3_SeriesResistor.h>

using namespace CPS;

EMT::Ph3::SeriesResistor::SeriesResistor(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {

	mPhaseType = PhaseType::ABC;
	setTerminalNumber(2);

	addAttribute<Real>("R", &mResistance, Flags::read | Flags::write);
}

PowerComponent<Real>::Ptr EMT::Ph3::SeriesResistor::clone(String name) {
	auto copy = SeriesResistor::make(name, mLogLevel);
	copy->setParameters(mResistance);
	return copy;
}

void EMT::Ph3::SeriesResistor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Complex phasorA = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfVoltage(0,0) = phasorA.real();
	Complex alpha(cos(2./3.*PI), sin(2./3.*PI));
	mIntfVoltage(1, 0) = Complex(phasorA * pow(alpha,2)).real();
	mIntfVoltage(2, 0) = Complex(phasorA * alpha).real();

	mIntfCurrent = mIntfVoltage / mResistance;

	mSLog->info("\n--- Initialization from powerflow ---"
		"\nVoltage across amplitude and phase: \n{}"
		"\nCurrent amplitude and phase: \n{}"
		"\nTerminal 0 voltage amplitude and phase: \n{}"
		"\nTerminal 1 voltage amplitude and phase: \n{}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorMatrixToString(mIntfVoltage),
		Logger::phasorMatrixToString(mIntfCurrent),
		Logger::phasorMatrixToString(initialVoltage(0)),
		Logger::phasorMatrixToString(initialVoltage(1)));
}

void EMT::Ph3::SeriesResistor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph3::SeriesResistor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Real conductance = 1./mResistance;

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
		mSLog->info("Add {} to {}, {}", conductance, simNode(0,0), simNode(0,0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {} to {}, {}", conductance, simNode(1,0), simNode(1,0));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {} to {}, {}", -conductance, simNode(0,0), simNode(1,0));
		mSLog->info("Add {} to {}, {}", -conductance, simNode(1,0), simNode(0,0));
	}
}

void EMT::Ph3::SeriesResistor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mResistor.mnaUpdateVoltage(*mLeftVector);
	mResistor.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::SeriesResistor::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage = Matrix::Zero(3,1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, simNode(1,0));
		mIntfVoltage(1,0) = Math::realFromVectorElement(leftVector, simNode(1,1));
		mIntfVoltage(2,0) = Math::realFromVectorElement(leftVector, simNode(1,2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::realFromVectorElement(leftVector, simNode(0,0));
		mIntfVoltage(1,0) = mIntfVoltage(1,0) - Math::realFromVectorElement(leftVector, simNode(0,1));
		mIntfVoltage(2,0) = mIntfVoltage(2,0) - Math::realFromVectorElement(leftVector, simNode(0,2));
	}

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage A: {}", mIntfVoltage(0,0));
}

void EMT::Ph3::SeriesResistor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mIntfVoltage / mResistance;

	SPDLOG_LOGGER_DEBUG(mSLog, "Current A: {} < {}", mIntfCurrent(0,0));
}
