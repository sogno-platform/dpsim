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

#include <cps/EMT/EMT_Ph1_Resistor.h>

using namespace CPS;

EMT::Ph1::Resistor::Resistor(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(1,1);
	mIntfCurrent = Matrix::Zero(1,1);

	addAttribute<Real>("R", &mResistance, Flags::read | Flags::write);
}

PowerComponent<Real>::Ptr EMT::Ph1::Resistor::clone(String name) {
	auto copy = Resistor::make(name, mLogLevel);
	copy->setParameters(mResistance);
	return copy;
}

void EMT::Ph1::Resistor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	mIntfVoltage(0,0) = (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / mResistance;

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:f}"
		"\nCurrent: {:f}"
		"\nTerminal 0 voltage: {:f}"
		"\nTerminal 1 voltage: {:f}"
		"\n--- Initialization from powerflow finished ---",
		mIntfVoltage(0,0),
		mIntfCurrent(0,0),
		initialSingleVoltage(0).real(),
		initialSingleVoltage(1).real());
}

void EMT::Ph1::Resistor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph1::Resistor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Real conductance = 1. / mResistance;
	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0)  &&  terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), -conductance);
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), -conductance);
	}

	if (terminalNotGrounded(0))
		mSLog->info("Add {:f} to system at ({:d},{:d})", conductance, simNode(0), simNode(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:f} to system at ({:d},{:d})", conductance, simNode(1), simNode(1));
	if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
		mSLog->info("Add {:f} to system at ({:d},{:d})", -conductance, simNode(0), simNode(1));
		mSLog->info("Add {:f} to system at ({:d},{:d})", -conductance, simNode(1), simNode(0));
	}
}

void EMT::Ph1::Resistor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mResistor.mnaUpdateVoltage(*mLeftVector);
	mResistor.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph1::Resistor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage(0,0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, simNode(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::realFromVectorElement(leftVector, simNode(0));
}

void EMT::Ph1::Resistor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / mResistance;
}
