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

#include <cps/DP/DP_Ph1_Switch.h>

using namespace CPS;

DP::Ph1::Switch::Switch(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Real>("R_open", &mOpenResistance, Flags::read | Flags::write);
	addAttribute<Real>("R_closed", &mClosedResistance, Flags::read | Flags::write);
	addAttribute<Bool>("is_closed", &mIsClosed, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph1::Switch::clone(String name) {
	auto copy = Switch::make(name, mLogLevel);
	copy->setParameters(mOpenResistance, mClosedResistance, mIsClosed);
	return copy;
}

void DP::Ph1::Switch::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real impedance = (mIsClosed) ? mClosedResistance : mOpenResistance;
	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void DP::Ph1::Switch::initialize(Matrix frequencies) {
	PowerComponent<Complex>::initialize(frequencies);
}

void DP::Ph1::Switch::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void DP::Ph1::Switch::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Complex conductance = (mIsClosed) ?
		Complex( 1./mClosedResistance, 0 ) : Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), -conductance);
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), -conductance);
	}

	mSLog->info("-- Stamp ---");
	if (terminalNotGrounded(0))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), simNode(0), simNode(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), simNode(1), simNode(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), simNode(0), simNode(1));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), simNode(1), simNode(0));
	}
}

void DP::Ph1::Switch::mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed) {
	Complex conductance = (closed) ?
		Complex( 1./mClosedResistance, 0 ) :
		Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), conductance);

	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), -conductance);
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), -conductance);
	}

	mSLog->info("-- Stamp ---");
	if (terminalNotGrounded(0))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), simNode(0), simNode(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), simNode(1), simNode(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), simNode(0), simNode(1));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), simNode(1), simNode(0));
	}
}

void DP::Ph1::Switch::mnaApplyRightSideVectorStamp(Matrix& rightVector) { }

void DP::Ph1::Switch::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSwitch.mnaUpdateVoltage(*mLeftVector);
	mSwitch.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::Switch::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1)) mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(1,0));
	if (terminalNotGrounded(0)) mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, simNode(0));
}

void DP::Ph1::Switch::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = (mIsClosed) ?
		mIntfVoltage(0,0) / mClosedResistance :
		mIntfVoltage(0,0) / mOpenResistance;
}
