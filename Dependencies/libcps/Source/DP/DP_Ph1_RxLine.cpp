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

#include <cps/DP/DP_Ph1_RxLine.h>

using namespace CPS;

DP::Ph1::RxLine::RxLine(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Real>("R", &mSeriesRes, Flags::read | Flags::write);
	addAttribute<Real>("L", &mSeriesInd, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph1::RxLine::clone(String name) {
	auto copy = RxLine::make(name, mLogLevel);
	copy->setParameters(mSeriesRes, mSeriesInd);
	return copy;
}

void DP::Ph1::RxLine::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	Complex impedance = { mSeriesRes, mSeriesInd * 2.*PI * frequency };
	mIntfCurrent(0, 0) = mVoltage / impedance;
	mVirtualNodes[0]->setInitialVoltage( initialSingleVoltage(0) + mIntfCurrent(0, 0) * mSeriesRes );

	// Default model with virtual node in between
	mSubResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_res", mLogLevel);
	mSubResistor->setParameters(mSeriesRes);
	mSubResistor->connect({ mTerminals[0]->node(), mVirtualNodes[0] });
	mSubResistor->initializeFromPowerflow(frequency);

	mSubInductor = std::make_shared<DP::Ph1::Inductor>(mName + "_ind", mLogLevel);
	mSubInductor->setParameters(mSeriesInd);
	mSubInductor->connect({ mVirtualNodes[0], mTerminals[1]->node() });
	mSubInductor->initializeFromPowerflow(frequency);

	mInitialResistor = std::make_shared<DP::Ph1::Resistor>(mName + "_snubber_res", mLogLevel);
	mInitialResistor->setParameters(1e6);
	mInitialResistor->connect({ Node::GND, mTerminals[1]->node() });
	mInitialResistor->initializeFromPowerflow(frequency);

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

void DP::Ph1::RxLine::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	mSubInductor->mnaInitialize(omega, timeStep, leftVector);
	mSubResistor->mnaInitialize(omega, timeStep, leftVector);
	mInitialResistor->mnaInitialize(omega, timeStep, leftVector);
	for (auto task : mSubInductor->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	for (auto task : mSubResistor->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::RxLine::mnaApplyInitialSystemMatrixStamp(Matrix& systemMatrix) {
	mInitialResistor->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::RxLine::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubResistor->mnaApplySystemMatrixStamp(systemMatrix);
	mSubInductor->mnaApplySystemMatrixStamp(systemMatrix);
	mInitialResistor->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::RxLine::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubResistor->mnaApplyRightSideVectorStamp(rightVector);
	mSubInductor->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::RxLine::MnaPreStep::execute(Real time, Int timeStepCount) {
	mLine.mnaApplyRightSideVectorStamp(mLine.mRightVector);
}

void DP::Ph1::RxLine::MnaPostStep::execute(Real time, Int timeStepCount) {
	mLine.mnaUpdateVoltage(*mLeftVector);
	mLine.mnaUpdateCurrent(*mLeftVector);
}

void DP::Ph1::RxLine::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, simNode(0));
}

void DP::Ph1::RxLine::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = mSubInductor->intfCurrent()(0,0);
}
