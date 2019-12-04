/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 *         Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
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

#include <cps/EMT/EMT_Ph3_Switch.h>
#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))

using namespace CPS;

EMT::Ph3::Switch::Switch(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(1,1);
	mIntfCurrent = Matrix::Zero(1,1);

	addAttribute<Matrix>("R_open", &mOpenResistance, Flags::read | Flags::write);
	addAttribute<Matrix>("R_closed", &mClosedResistance, Flags::read | Flags::write);
	addAttribute<Bool>("is_closed", &mSwitchClosed, Flags::read | Flags::write);
}

PowerComponent<Real>::Ptr EMT::Ph3::Switch::clone(String name) {
	auto copy = Switch::make(name, mLogLevel);
	copy->setParameters(mOpenResistance, mClosedResistance, mSwitchClosed);
	return copy;
}

void EMT::Ph3::Switch::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Matrix impedance = (mSwitchClosed) ? mClosedResistance : mOpenResistance;
	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	mIntfVoltage = vInitABC.real();
	mIntfCurrent = (impedance.inverse() * vInitABC).real();

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(mIntfVoltage),
		Logger::matrixToString(mIntfCurrent),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void EMT::Ph3::Switch::initialize(Matrix frequencies) {
	PowerComponent<Real>::initialize(frequencies);
}

void EMT::Ph3::Switch::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph3::Switch::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Matrix conductance = (mSwitchClosed) ?
		mClosedResistance.inverse() : mOpenResistance.inverse();

	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), conductance(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), conductance(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), -conductance(2, 2));


		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), -conductance(2, 2));
	}
	mSLog->info(
		"\nConductance matrix: {:s}",
		Logger::matrixToString(conductance));
}

void EMT::Ph3::Switch::mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed) {
	Matrix conductance = (closed) ?
		mClosedResistance.inverse() : mOpenResistance.inverse();

	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), conductance(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), conductance(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), -conductance(2, 2));


		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), -conductance(2, 2));
	}

	mSLog->info(
		"\nConductance matrix: {:s}",
		Logger::matrixToString(conductance));
}

void EMT::Ph3::Switch::mnaApplyRightSideVectorStamp(Matrix& rightVector) { }

void EMT::Ph3::Switch::MnaPostStep::execute(Real time, Int timeStepCount) {
	mSwitch.mnaUpdateVoltage(*mLeftVector);
	mSwitch.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::Switch::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, simNode(1, 0));
		mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, simNode(1, 1));
		mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, simNode(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::realFromVectorElement(leftVector, simNode(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::realFromVectorElement(leftVector, simNode(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::realFromVectorElement(leftVector, simNode(0, 2));
	}
}

void EMT::Ph3::Switch::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = (mSwitchClosed) ?
		mClosedResistance.inverse() * mIntfVoltage:
		mOpenResistance.inverse() *mIntfVoltage;
}
