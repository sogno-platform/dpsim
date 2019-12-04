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

#include <cps/EMT/EMT_Ph3_VoltageSourceNorton.h>

using namespace CPS;

EMT::Ph3::VoltageSourceNorton::VoltageSourceNorton(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<Complex>("V_ref", &mVoltageRef, Flags::read | Flags::write);
	addAttribute<Real>("R", &mResistance, Flags::read | Flags::write);
}

void EMT::Ph3::VoltageSourceNorton::setParameters(Complex voltageRef, Real srcFreq,  Real resistance) {

	Base::Ph1::VoltageSource::setParameters(voltageRef, srcFreq);
	mResistance = resistance;
	mConductance = 1. / mResistance;
}

PowerComponent<Real>::Ptr EMT::Ph3::VoltageSourceNorton::clone(String name) {
	auto copy = VoltageSourceNorton::make(name, mLogLevel);
	copy->setParameters(mVoltageRef, mSrcFreq, mResistance);
	return copy;
}

void EMT::Ph3::VoltageSourceNorton::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateSimNodes();
	Complex voltageRef = attribute<Complex>("V_ref")->get();
	mIntfVoltage(0, 0) = voltageRef.real() * cos(Math::phase(voltageRef));
	mIntfVoltage(1, 0) = voltageRef.real() * cos(Math::phase(voltageRef) - 2. / 3. * M_PI);
	mIntfVoltage(2, 0) = voltageRef.real() * cos(Math::phase(voltageRef) + 2. / 3. * M_PI);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::VoltageSourceNorton::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Apply matrix stamp for equivalent resistance
	if (terminalNotGrounded(0)){
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(0, 0), mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(0, 1), mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(0, 2), mConductance);
	}
	if (terminalNotGrounded(1)){
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(1, 0), mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(1, 1), mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(1, 2), mConductance);
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), simNode(1, 0), -mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), simNode(0, 0), -mConductance);

		Math::addToMatrixElement(systemMatrix, simNode(0, 1), simNode(1, 1), -mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(1, 1), simNode(0, 1), -mConductance);

		Math::addToMatrixElement(systemMatrix, simNode(0, 2), simNode(1, 2), -mConductance);
		Math::addToMatrixElement(systemMatrix, simNode(1, 2), simNode(0, 2), -mConductance);
	}
}

void EMT::Ph3::VoltageSourceNorton::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Apply matrix stamp for equivalent current source
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, simNode(0, 0), -mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(0, 1), -mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(0, 2), -mEquivCurrent(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, simNode(1, 0), mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, simNode(1, 1), mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, simNode(1, 2), mEquivCurrent(2, 0));
	}
}

void EMT::Ph3::VoltageSourceNorton::updateState(Real time) {
	// Check if set source was called
	if (Math::abs(mVoltageRef) > 0) {
		mIntfVoltage(0, 0) = Math::abs(mVoltageRef) * cos(2. * PI * mSrcFreq * time + Math::phase(mVoltageRef));
		mIntfVoltage(1, 0) = Math::abs(mVoltageRef) * cos(2. * PI * mSrcFreq * time + Math::phase(mVoltageRef) - 2. / 3. * M_PI);
		mIntfVoltage(2, 0) = Math::abs(mVoltageRef) * cos(2. * PI * mSrcFreq * time + Math::phase(mVoltageRef) + 2. / 3. * M_PI);
	}

	mEquivCurrent = mIntfVoltage / mResistance;
}


void EMT::Ph3::VoltageSourceNorton::mnaUpdateVoltage(const Matrix& leftVector) {
	// Calculate v1 - v0
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

void EMT::Ph3::VoltageSourceNorton::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSourceNorton.updateState(time);
	mVoltageSourceNorton.mnaApplyRightSideVectorStamp(mVoltageSourceNorton.mRightVector);
}

void EMT::Ph3::VoltageSourceNorton::MnaPostStep::execute(Real time, Int timeStepCount) {
	mVoltageSourceNorton.mnaUpdateVoltage(*mLeftVector);
	mVoltageSourceNorton.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::VoltageSourceNorton::mnaUpdateCurrent(const Matrix& leftVector) {
	// signs are not verified
	mIntfCurrent(0, 0) = mEquivCurrent(0, 0) - mIntfVoltage(0, 0) / mResistance;
	mIntfCurrent(1, 0) = mEquivCurrent(1, 0) - mIntfVoltage(1, 0) / mResistance;
	mIntfCurrent(2, 0) = mEquivCurrent(2, 0) - mIntfVoltage(2, 0) / mResistance;
}
