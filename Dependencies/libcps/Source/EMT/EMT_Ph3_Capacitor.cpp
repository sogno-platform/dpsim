/**
 * @file
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

#include <cps/EMT/EMT_Ph3_Capacitor.h>
#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))

using namespace CPS;

EMT::Ph3::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	mEquivCurrent = Matrix::Zero(3, 1);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);
	setTerminalNumber(2);

	addAttribute<Matrix>("C", &mCapacitance, Flags::read | Flags::write);
}


PowerComponent<Real>::Ptr EMT::Ph3::Capacitor::clone(String name) {
	auto copy = Capacitor::make(name, mLogLevel);
	copy->setParameters(mCapacitance);
	return copy;
}
void EMT::Ph3::Capacitor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real omega = 2 * PI * frequency;
	MatrixComp admittance = MatrixComp::Zero(3, 3);
	admittance <<
		Complex(0, omega * mCapacitance(0, 0)), Complex(0, omega * mCapacitance(0, 1)), Complex(0, omega * mCapacitance(0, 2)),
		Complex(0, omega * mCapacitance(1, 0)), Complex(0, omega * mCapacitance(1, 1)), Complex(0, omega * mCapacitance(1, 2)),
		Complex(0, omega * mCapacitance(2, 0)), Complex(0, omega * mCapacitance(2, 1)), Complex(0, omega * mCapacitance(2, 2));

	MatrixComp vInitABC = Matrix::Zero(3, 1);
	vInitABC(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	mIntfVoltage = vInitABC.real();
	mIntfCurrent = (admittance * vInitABC).real();


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

void EMT::Ph3::Capacitor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	mEquivCond = (2.0 * mCapacitance) / timeStep;
	// Update internal state
	mEquivCurrent = - mIntfCurrent + - mEquivCond * mIntfVoltage;

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

}

void EMT::Ph3::Capacitor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 1), mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 2), mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 0), mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 2), mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 0), mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 1), mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), mEquivCond(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 1), mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 2), mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 0), mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 2), mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 0), mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 1), mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), mEquivCond(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), -mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 1), -mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 2), -mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 0), -mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), -mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 2), -mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 0), -mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 1), -mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), -mEquivCond(2, 2));


		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), -mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 1), -mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 2), -mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 0), -mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), -mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 2), -mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 0), -mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 1), -mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), -mEquivCond(2, 2));

		mSLog->info(
			"\nEquivalent Conductance: {:s}",
			Logger::matrixToString(mEquivCond));
	}
}

void EMT::Ph3::Capacitor::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mEquivCurrent = -mIntfCurrent + -mEquivCond * mIntfVoltage;
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, simNode(0, 0), mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(0, 1), mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(0, 2), mEquivCurrent(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, simNode(1, 0), -mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(1, 1), -mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(1, 2), -mEquivCurrent(2, 0));
	}
	mSLog->debug(
		"\nEquivalent Current: {:s}",
		Logger::matrixToString(mEquivCurrent));
}

void EMT::Ph3::Capacitor::MnaPreStep::execute(Real time, Int timeStepCount) {
	mCapacitor.mnaApplyRightSideVectorStamp(mCapacitor.mRightVector);
}

void EMT::Ph3::Capacitor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mCapacitor.mnaUpdateVoltage(*mLeftVector);
	mCapacitor.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::Capacitor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage = Matrix::Zero(3,1);
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

void EMT::Ph3::Capacitor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mEquivCond * mIntfVoltage + mEquivCurrent;
	mSLog->debug(
		"\nCurrent: {:s}",
		Logger::matrixToString(mIntfCurrent)
	);
}

