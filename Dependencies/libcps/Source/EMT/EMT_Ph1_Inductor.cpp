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

#include <cps/EMT/EMT_Ph1_Inductor.h>

using namespace CPS;

EMT::Ph1::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	mEquivCurrent = 0;
	mIntfVoltage = Matrix::Zero(1,1);
	mIntfCurrent = Matrix::Zero(1,1);
	setTerminalNumber(2);

	addAttribute<Real>("L", &mInductance, Flags::read | Flags::write);
}

PowerComponent<Real>::Ptr EMT::Ph1::Inductor::clone(String name) {
	auto copy = Inductor::make(name, mLogLevel);
	copy->setParameters(mInductance);
	return copy;
}

void EMT::Ph1::Inductor::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	Real omega = 2 * PI * frequency;
	Complex impedance = { 0, omega * mInductance };
	mIntfVoltage(0,0) = (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
	mIntfCurrent(0,0) = (mIntfVoltage(0,0) / impedance).real();

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

void EMT::Ph1::Inductor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mEquivCond = timeStep / (2.0 * mInductance);
	// Update internal state
	mEquivCurrent = mEquivCond * mIntfVoltage(0,0) + mIntfCurrent(0,0);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph1::Inductor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), mEquivCond);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), mEquivCond);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), -mEquivCond);
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), -mEquivCond);
	}
}

void EMT::Ph1::Inductor::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Update internal state
	mEquivCurrent = mEquivCond * mIntfVoltage(0,0) + mIntfCurrent(0,0);
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, simNode(0), mEquivCurrent);
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, simNode(1), -mEquivCurrent);
}

void EMT::Ph1::Inductor::MnaPreStep::execute(Real time, Int timeStepCount) {
	mInductor.mnaApplyRightSideVectorStamp(mInductor.mRightVector);
}

void EMT::Ph1::Inductor::MnaPostStep::execute(Real time, Int timeStepCount) {
	mInductor.mnaUpdateVoltage(*mLeftVector);
	mInductor.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph1::Inductor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage(0,0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, simNode(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::realFromVectorElement(leftVector, simNode(0));
}

void EMT::Ph1::Inductor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = mEquivCond * mIntfVoltage(0,0) + mEquivCurrent;
}

