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

#include <cps/EMT/EMT_Ph3_VoltageSource.h>


using namespace CPS;

EMT::Ph3::VoltageSource::VoltageSource(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(3, 1);
	mIntfCurrent = Matrix::Zero(3, 1);

	addAttribute<Complex>("V_ref", Flags::read | Flags::write);
	addAttribute<Real>("f_src", Flags::read | Flags::write);
}

void EMT::Ph3::VoltageSource::setParameters(Complex voltageRef, Real srcFreq) {
	attribute<Complex>("V_ref")->set(voltageRef);
	attribute<Real>("f_src")->set(srcFreq);

	parametersSet = true;
}

PowerComponent<Real>::Ptr EMT::Ph3::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("V_ref")->get(), attribute<Real>("f_src")->get());
	return copy;
}


void EMT::Ph3::VoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);

	updateSimNodes();
	mVoltageRef = attribute<Complex>("V_ref");
	mSrcFreq = attribute<Real>("f_src");
	mIntfVoltage(0, 0) = Math::abs(mVoltageRef->get()) * cos(Math::phase(mVoltageRef->get()));
	mIntfVoltage(1, 0) = Math::abs(mVoltageRef->get()) * cos(Math::phase(mVoltageRef->get()) - 2. / 3. * M_PI);
	mIntfVoltage(2, 0) = Math::abs(mVoltageRef->get()) * cos(Math::phase(mVoltageRef->get()) + 2. / 3. * M_PI);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::VoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNode(0, 0), mVirtualNodes[0]->simNode(PhaseType::A), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::A), simNode(0, 0), -1);

		Math::addToMatrixElement(systemMatrix, simNode(0, 1), mVirtualNodes[0]->simNode(PhaseType::B), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::B), simNode(0, 1), -1);

		Math::addToMatrixElement(systemMatrix, simNode(0, 2), mVirtualNodes[0]->simNode(PhaseType::C), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::C), simNode(0, 2), -1);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(1, 0), mVirtualNodes[0]->simNode(PhaseType::A), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::A), simNode(1, 0), 1);

		Math::addToMatrixElement(systemMatrix, simNode(1, 1), mVirtualNodes[0]->simNode(PhaseType::B), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::B), simNode(1, 1), 1);

		Math::addToMatrixElement(systemMatrix, simNode(1, 2), mVirtualNodes[0]->simNode(PhaseType::C), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(PhaseType::C), simNode(1, 2), 1);
	}


	// if (terminalNotGrounded(0)) {
	// 	mLog.debug() << "Add " << -1 << " to " << simNode(0) << "," << mVirtualNodes[0]->simNode() << std::endl;
	// 	mLog.debug() << "Add " << -1 << " to " << mVirtualNodes[0]->simNode() << "," << simNode(0) << std::endl;
	// }
	// if (terminalNotGrounded(1)) {
	// 	mLog.debug() << "Add " << 1 << " to " << simNode(1) << "," << mVirtualNodes[0]->simNode() << std::endl;
	// 	mLog.debug() << "Add " << 1 << " to " << mVirtualNodes[0]->simNode() << "," << simNode(1) << std::endl;
	// }
}

void EMT::Ph3::VoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(PhaseType::A), mIntfVoltage(0, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(PhaseType::B), mIntfVoltage(1, 0));
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(PhaseType::C), mIntfVoltage(2, 0));
}

void EMT::Ph3::VoltageSource::updateVoltage(Real time) {
	Complex voltageRef = mVoltageRef->get();
	Real srcFreq = mSrcFreq->get();
	if (srcFreq > 0) {
		mIntfVoltage(0, 0) = Math::abs(voltageRef) * cos(time * 2. * PI * srcFreq + Math::phase(voltageRef));
		mIntfVoltage(1, 0) = Math::abs(voltageRef) * cos(time * 2. * PI * srcFreq + Math::phase(voltageRef) - 2. / 3. * M_PI);
		mIntfVoltage(2, 0) = Math::abs(voltageRef) * cos(time * 2. * PI * srcFreq + Math::phase(voltageRef) + 2. / 3. * M_PI);
	}
	else {
		mIntfVoltage(0, 0) = voltageRef.real();
		mIntfVoltage(1, 0) = voltageRef.real();
		mIntfVoltage(2, 0) = voltageRef.real();
	}
}

void EMT::Ph3::VoltageSource::updateVoltage(Matrix vabc) {
	mIntfVoltage=vabc;
}

void EMT::Ph3::VoltageSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.updateVoltage(time);
	mVoltageSource.mnaApplyRightSideVectorStamp(mVoltageSource.mRightVector);
}

void EMT::Ph3::VoltageSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void EMT::Ph3::VoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->simNode(PhaseType::A));
	mIntfCurrent(1, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->simNode(PhaseType::B));
	mIntfCurrent(2, 0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->simNode(PhaseType::C));
}
