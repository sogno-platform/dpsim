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

#include <cps/EMT/EMT_Ph1_VoltageSourceNorton.h>

using namespace CPS;

EMT::Ph1::VoltageSourceNorton::VoltageSourceNorton(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(1,1);
	mIntfCurrent = Matrix::Zero(1,1);

	addAttribute<Complex>("V_ref", &mVoltageRef, Flags::read | Flags::write);
	addAttribute<Real>("R", &mResistance, Flags::read | Flags::write);
}

PowerComponent<Real>::Ptr EMT::Ph1::VoltageSourceNorton::clone(String name) {
	auto copy = VoltageSourceNorton::make(name, mLogLevel);
	copy->setParameters(mVoltageRef, mSrcFreq, mResistance);
	return copy;
}

void EMT::Ph1::VoltageSourceNorton::setParameters(Complex voltage, Real srcFreq, Real resistance) {
	Base::Ph1::VoltageSource::setParameters(voltage, srcFreq);

	mResistance = resistance;
	mConductance = 1. / mResistance;
}

void EMT::Ph1::VoltageSourceNorton::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mIntfVoltage(0, 0) = attributeComplex("V_ref")->get().real();
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph1::VoltageSourceNorton::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Apply matrix stamp for equivalent resistance
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), mConductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), mConductance);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), -mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), -mConductance);
	}
}

void EMT::Ph1::VoltageSourceNorton::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Apply matrix stamp for equivalent current source
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, simNode(0), -mEquivCurrent);
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, simNode(1), mEquivCurrent);
}

void EMT::Ph1::VoltageSourceNorton::updateState(Real time) {
	// Check if set source was called
	if (Math::abs(mVoltageRef)  > 0)
		mIntfVoltage(0,0) = Math::abs(mVoltageRef) * cos(2.*PI*mSrcFreq*time + Math::phase(mVoltageRef));

	mEquivCurrent = mIntfVoltage(0,0) / mResistance;
}

void EMT::Ph1::VoltageSourceNorton::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.updateState(time);
	mVoltageSource.mnaApplyRightSideVectorStamp(mVoltageSource.mRightVector);
}

void EMT::Ph1::VoltageSourceNorton::MnaPostStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.mnaUpdateVoltage(*mLeftVector);
	mVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph1::VoltageSourceNorton::mnaUpdateVoltage(const Matrix& leftVector) {
	// Calculate v1 - v0
	if (terminalNotGrounded(1))
		mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, simNode(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::realFromVectorElement(leftVector, simNode(0));
}

void EMT::Ph1::VoltageSourceNorton::mnaUpdateCurrent(const Matrix& leftVector) {
	// TODO: verify signs
	mIntfCurrent(0,0) = mEquivCurrent - mIntfVoltage(0,0) / mResistance;
}
