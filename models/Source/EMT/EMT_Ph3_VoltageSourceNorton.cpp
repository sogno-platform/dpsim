/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_VoltageSourceNorton.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::VoltageSourceNorton::VoltageSourceNorton(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
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

	mParametersSet = true;
}

SimPowerComp<Real>::Ptr EMT::Ph3::VoltageSourceNorton::clone(String name) {
	auto copy = VoltageSourceNorton::make(name, mLogLevel);
	copy->setParameters(mVoltageRef, mSrcFreq, mResistance);
	return copy;
}

void EMT::Ph3::VoltageSourceNorton::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
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
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), mConductance);
	}
	if (terminalNotGrounded(1)){
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), mConductance);
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -mConductance);

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), -mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), -mConductance);

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), -mConductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), -mConductance);
	}
}

void EMT::Ph3::VoltageSourceNorton::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Apply matrix stamp for equivalent current source
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), -mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 1), -mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 2), -mEquivCurrent(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 1), mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 2), mEquivCurrent(2, 0));
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
		mIntfVoltage(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
		mIntfVoltage(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
		mIntfVoltage(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		mIntfVoltage(1, 0) = mIntfVoltage(1, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		mIntfVoltage(2, 0) = mIntfVoltage(2, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
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
