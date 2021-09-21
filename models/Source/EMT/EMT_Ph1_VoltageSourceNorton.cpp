/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph1_VoltageSourceNorton.h>

using namespace CPS;

EMT::Ph1::VoltageSourceNorton::VoltageSourceNorton(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	setTerminalNumber(2);
	mIntfVoltage = Matrix::Zero(1,1);
	mIntfCurrent = Matrix::Zero(1,1);

	addAttribute<Complex>("V_ref", &mVoltageRef, Flags::read | Flags::write);
	addAttribute<Real>("R", &mResistance, Flags::read | Flags::write);
}

SimPowerComp<Real>::Ptr EMT::Ph1::VoltageSourceNorton::clone(String name) {
	auto copy = VoltageSourceNorton::make(name, mLogLevel);
	copy->setParameters(mVoltageRef, mSrcFreq, mResistance);
	return copy;
}

void EMT::Ph1::VoltageSourceNorton::setParameters(Complex voltage, Real srcFreq, Real resistance) {
	Base::Ph1::VoltageSource::setParameters(voltage, srcFreq);

	mResistance = resistance;
	mConductance = 1. / mResistance;

	mParametersSet = true;
}

void EMT::Ph1::VoltageSourceNorton::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mIntfVoltage(0, 0) = attributeComplex("V_ref")->get().real();
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph1::VoltageSourceNorton::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Apply matrix stamp for equivalent resistance
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mConductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mConductance);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mConductance);
	}
}

void EMT::Ph1::VoltageSourceNorton::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Apply matrix stamp for equivalent current source
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, matrixNodeIndex(0), -mEquivCurrent);
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, matrixNodeIndex(1), mEquivCurrent);
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
		mIntfVoltage(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::VoltageSourceNorton::mnaUpdateCurrent(const Matrix& leftVector) {
	// TODO: verify signs
	mIntfCurrent(0,0) = mEquivCurrent - mIntfVoltage(0,0) / mResistance;
}
