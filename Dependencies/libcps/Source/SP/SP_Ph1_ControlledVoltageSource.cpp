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

#include <cps/SP/SP_Ph1_ControlledVoltageSource.h>

using namespace CPS;

SP::Ph1::ControlledVoltageSource::ControlledVoltageSource(String uid, String name, Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
}

void SP::Ph1::ControlledVoltageSource::setParameters(MatrixComp voltageRef) {
	mIntfVoltage = voltageRef;
	parametersSet = true;
}

PowerComponent<Complex>::Ptr SP::Ph1::ControlledVoltageSource::clone(String name) {
	auto copy = ControlledVoltageSource::make(name, mLogLevel);
	copy->setParameters(attribute<Matrix>("v_intf")->get());
	return copy;
}


void SP::Ph1::ControlledVoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateSimNodes();
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void SP::Ph1::ControlledVoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, simNode(0), mVirtualNodes[0]->simNode(), Complex(-1, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(0), Complex(-1, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, simNode(1), mVirtualNodes[0]->simNode(), Complex(1, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->simNode(), simNode(1), Complex(1, 0));
	}
/*
	if (terminalNotGrounded(0)) {
		mLog.debug() << "Add " << -1 << " to " << simNode(0) << "," << mVirtualNodes[0]->simNode() << std::endl;
		mLog.debug() << "Add " << -1 << " to " << mVirtualNodes[0]->simNode() << "," << simNode(0) << std::endl;
	}
	if (terminalNotGrounded(1)) {
		mLog.debug() << "Add " << 1 << " to " << simNode(1) << "," << mVirtualNodes[0]->simNode() << std::endl;
		mLog.debug() << "Add " << 1 << " to " << mVirtualNodes[0]->simNode() << "," << simNode(1) << std::endl;
	}*/
}

void SP::Ph1::ControlledVoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->simNode(), mIntfVoltage(0, 0));
	mSLog->debug( "Add {:s} to source vector at {:d}",
		Logger::complexToString(mIntfVoltage(0, 0)), mVirtualNodes[0]->simNode());

}



void SP::Ph1::ControlledVoltageSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mControlledVoltageSource.mnaApplyRightSideVectorStamp(mControlledVoltageSource.mRightVector);
}

void SP::Ph1::ControlledVoltageSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mControlledVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::ControlledVoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->simNode());
}
