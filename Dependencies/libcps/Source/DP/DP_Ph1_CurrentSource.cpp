/** Current source
 *
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

#include <cps/DP/DP_Ph1_CurrentSource.h>

using namespace CPS;

DP::Ph1::CurrentSource::CurrentSource(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("I_ref", Flags::read | Flags::write);
}

DP::Ph1::CurrentSource::CurrentSource(String name, Complex current, Logger::Level logLevel)
	: CurrentSource(name, logLevel) {
	setParameters(current);
}

void DP::Ph1::CurrentSource::setParameters(Complex current) {
	attribute<Complex>("I_ref")->set(current);
	parametersSet = true;
}

PowerComponent<Complex>::Ptr DP::Ph1::CurrentSource::clone(String name) {
	auto copy = CurrentSource::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("I_ref")->get());
	return copy;
}

void DP::Ph1::CurrentSource::initializeFromPowerflow(Real frequency) {
	checkForUnconnectedTerminals();

	mIntfVoltage(0,0) = initialSingleVoltage(0) - initialSingleVoltage(1);
	mCurrentRef = attribute<Complex>("I_ref");
	mIntfCurrent(0,0) = mCurrentRef->get();

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

void DP::Ph1::CurrentSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mCurrentRef = attribute<Complex>("I_ref");
	mIntfCurrent(0,0) = mCurrentRef->get();
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::CurrentSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mCurrentSource.mnaApplyRightSideVectorStamp(mCurrentSource.mRightVector);
}

void DP::Ph1::CurrentSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mIntfCurrent(0,0) = mCurrentRef->get();

	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, simNode(0), -mIntfCurrent(0,0));
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, simNode(1), mIntfCurrent(0,0));
}

void DP::Ph1::CurrentSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mCurrentSource.mnaUpdateVoltage(*mLeftVector);
}

void DP::Ph1::CurrentSource::mnaUpdateVoltage(const Matrix& leftVector) {
	mIntfVoltage(0,0) = 0;
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, simNode(0));
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, simNode(1));
}
