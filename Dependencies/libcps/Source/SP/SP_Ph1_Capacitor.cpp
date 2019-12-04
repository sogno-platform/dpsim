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

#include <cps/SP/SP_Ph1_Capacitor.h>

using namespace CPS;

SP::Ph1::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
	setTerminalNumber(2);

	addAttribute<Real>("C", &mCapacitance, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr SP::Ph1::Capacitor::clone(String name) {
	auto copy = Capacitor::make(name, mLogLevel);
	copy->setParameters(mCapacitance);
	return copy;
}

void SP::Ph1::Capacitor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real omega = 2 * PI * frequency;
	mSusceptance = Complex(0, omega * mCapacitance);
	mIntfVoltage(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent = mSusceptance * mIntfVoltage;

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


void SP::Ph1::Capacitor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateSimNodes();

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)));
}

void SP::Ph1::Capacitor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {

	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), mSusceptance);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), mSusceptance);
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), -mSusceptance);
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), -mSusceptance);
	}

	mSLog->info("-- Matrix Stamp ---");
	if (terminalNotGrounded(0))
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			mSusceptance.real(), mSusceptance.imag(), simNode(0), simNode(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			mSusceptance.real(), mSusceptance.imag(), simNode(1), simNode(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			-mSusceptance.real(), -mSusceptance.imag(), simNode(0), simNode(1));
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			-mSusceptance.real(), -mSusceptance.imag(), simNode(1), simNode(0));
	}
}

void SP::Ph1::Capacitor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mCapacitor.mnaUpdateVoltage(*mLeftVector);
	mCapacitor.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::Capacitor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, simNode(1));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, simNode(0));
	}
}

void SP::Ph1::Capacitor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSusceptance * mIntfVoltage;
}
