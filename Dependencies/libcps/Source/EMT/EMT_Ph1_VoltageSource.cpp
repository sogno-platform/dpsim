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

#include <cps/EMT/EMT_Ph1_VoltageSource.h>

using namespace CPS;

EMT::Ph1::VoltageSource::VoltageSource(String uid, String name,	Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(1,1);
	mIntfCurrent = Matrix::Zero(1,1);

	addAttribute<Complex>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

void EMT::Ph1::VoltageSource::setParameters(Complex voltageRef, Real srcFreq) {
	attribute<Complex>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);

	parametersSet = true;
}

PowerComponent<Real>::Ptr EMT::Ph1::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("V_ref")->get(), attribute<Real>("f_src")->get());
	return copy;
}

void EMT::Ph1::VoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();

	mVoltageRef = attribute<Complex>("V_ref");
	mSrcFreq = attribute<Real>("f_src");
	mIntfVoltage(0,0) = Math::abs(mVoltageRef->get()) * cos(Math::phase(mVoltageRef->get()));
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph1::VoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), -1);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(1), mVirtualNodes[0]->simNode(), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(1), 1);
	}

	if (terminalNotGrounded(0)) {
		mSLog->info("Add {:f} to system at ({:d},{:d})", -1, simNode(0), mVirtualNodes[0]->simNode());
		mSLog->info("Add {:f} to system at ({:d},{:d})", -1, mVirtualNodes[0]->simNode(), simNode(0));
	}
	if (terminalNotGrounded(1)) {
		mSLog->info("Add {:f} to system at ({:d},{:d})", 1, simNode(1), mVirtualNodes[0]->simNode());
		mSLog->info("Add {:f} to system at ({:d},{:d})", 1, mVirtualNodes[0]->simNode(), simNode(1));
	}
}

void EMT::Ph1::VoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0,0));
}

void EMT::Ph1::VoltageSource::updateVoltage(Real time) {
	Complex voltageRef = mVoltageRef->get();
	Real srcFreq = mSrcFreq->get();
	if (srcFreq > 0)
		mIntfVoltage(0,0) = Math::abs(voltageRef) * cos(time * 2.*PI*srcFreq + Math::phase(voltageRef));
	else
		mIntfVoltage(0,0) = voltageRef.real();
}

void EMT::Ph1::VoltageSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.updateVoltage(time);
	mVoltageSource.mnaApplyRightSideVectorStamp(mVoltageSource.mRightVector);
}

void EMT::Ph1::VoltageSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph1::VoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->simNode());
}
