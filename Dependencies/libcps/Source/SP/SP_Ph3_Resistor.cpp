/**
 * @file
 * @author Junjie Zhang <junjie.zhang@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/SP/SP_Ph3_Resistor.h>

using namespace CPS;


SP::Ph3::Resistor::Resistor(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(3, 1);
	mIntfCurrent = MatrixComp::Zero(3, 1);

	addAttribute<Matrix>("R", &mResistance, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr SP::Ph3::Resistor::clone(String name) {
	auto copy = Resistor::make(name, mLogLevel);
	copy->setParameters(mResistance);
	return copy;
}

void SP::Ph3::Resistor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real voltMag = Math::abs(mIntfVoltage(0, 0));
	Real voltPhase = Math::phase(mIntfVoltage(0, 0));
	mIntfVoltage(1, 0) = Complex(
		voltMag * cos(voltPhase - 2. / 3. * M_PI),
		voltMag * sin(voltPhase - 2. / 3. * M_PI));
	mIntfVoltage(2, 0) = Complex(
		voltMag * cos(voltPhase + 2. / 3. * M_PI),
		voltMag * sin(voltPhase + 2. / 3. * M_PI));
	mConductance = mResistance.inverse();
	mIntfCurrent = mConductance * mIntfVoltage;

	mSLog->info("Node 1 : {} <{}", Logger::phasorToString(initialVoltage(0)(0, 0)));
	mSLog->info("Node 2 : {} <{}", Logger::phasorToString(initialVoltage(1)(0, 0)));
	mSLog->info("initialize {} {} voltage to {} <{} and current to {} <{}",
		this->type(), this->name(),
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)));
}

void SP::Ph3::Resistor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateSimNodes();
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void SP::Ph3::Resistor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), Complex( mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 1), Complex( mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 2), Complex( mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 0), Complex( mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), Complex( mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 2), Complex( mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 0), Complex( mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 1), Complex( mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), Complex( mConductance(2, 2), 0));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), Complex( mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 1), Complex( mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 2), Complex( mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 0), Complex( mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), Complex( mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 2), Complex( mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 0), Complex( mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 1), Complex( mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), Complex( mConductance(2, 2), 0));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), - Complex( mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 1), - Complex( mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 2), - Complex( mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 0), - Complex( mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), - Complex( mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 2), - Complex( mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 0), - Complex( mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 1), - Complex( mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), - Complex( mConductance(2, 2), 0));


		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), - Complex( mConductance(0, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 1), - Complex( mConductance(0, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 2), - Complex( mConductance(0, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 0), - Complex( mConductance(1, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), - Complex( mConductance(1, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 2), - Complex( mConductance(1, 2), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 0), - Complex( mConductance(2, 0), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 1), - Complex( mConductance(2, 1), 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), - Complex( mConductance(2, 2), 0));
	}

	// TODO: add Log
	/*if (terminalNotGrounded(0))
		mLog.debug() << "Add " << conductance << " to " << simNode(0, 0) << "," << simNode(0, 0) << std::endl;
	if (terminalNotGrounded(1))
		mLog.debug() << "Add " << conductance << " to " << simNode(1, 0) << "," << simNode(1, 0) << std::endl;
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mLog.debug() << "Add " << -conductance << " to " << simNode(0, 0) << "," << simNode(1, 0) << std::endl;
		mLog.debug() << "Add " << -conductance << " to " << simNode(1, 0) << "," << simNode(0, 0) << std::endl;
	}*/
}

void SP::Ph3::Resistor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mResistor.mnaUpdateVoltage(*mLeftVector);
	mResistor.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph3::Resistor::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage = MatrixComp::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, simNode(1, 0));
		mIntfVoltage(1, 0) = Math::complexFromVectorElement(leftVector, simNode(1, 1));
		mIntfVoltage(2, 0) = Math::complexFromVectorElement(leftVector, simNode(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, simNode(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::complexFromVectorElement(leftVector, simNode(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::complexFromVectorElement(leftVector, simNode(0, 2));
	}

	//mLog.debug() << "Voltage A: " << std::abs(mIntfVoltage(0, 0))
	//	<< "<" << std::arg(mIntfVoltage(0, 0)) << std::endl;
}

void SP::Ph3::Resistor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mConductance * mIntfVoltage;
	//mLog.debug() << "Current A: " << std::abs(mIntfCurrent(0, 0))
	//	<< "<" << std::arg(mIntfCurrent(0, 0)) << std::endl;
}


void SP::Ph3::Resistor::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	// TODO
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, Complex(mResistance(0, 0), 0));
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, Complex(mResistance(1, 1), 0));
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, Complex(mResistance(2, 2), 0));
}
