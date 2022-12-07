/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_CurrentSource.h>

using namespace CPS;

EMT::Ph1::CurrentSource::CurrentSource(String uid, String name,	Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, logLevel),
	mCurrentRef(mAttributes->create<Complex>("I_ref")),
	mSrcFreq(mAttributes->create<Real>("f_src")) {
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);
}

SimPowerComp<Real>::Ptr EMT::Ph1::CurrentSource::clone(String name) {
	auto copy = CurrentSource::make(name, mLogLevel);
	copy->setParameters(**mCurrentRef, **mSrcFreq);
	return copy;
}

void EMT::Ph1::CurrentSource::setParameters(Complex currentRef, Real srcFreq) {
	**mCurrentRef = currentRef;
	**mSrcFreq = srcFreq;

	mParametersSet = true;
}

void EMT::Ph1::CurrentSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();
	(**mIntfCurrent)(0,0) = Math::abs(**mCurrentRef) * cos(Math::phase(**mCurrentRef));
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph1::CurrentSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, matrixNodeIndex(0), -(**mIntfCurrent)(0,0));

	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, matrixNodeIndex(1), (**mIntfCurrent)(0,0));
}

void EMT::Ph1::CurrentSource::updateState(Real time) {
	Complex currentRef = mCurrentRef->get();
	Real srcFreq = mSrcFreq->get();
	if (srcFreq > 0)
		(**mIntfCurrent)(0,0) = Math::abs(currentRef) * cos(time * 2.*PI*srcFreq + Math::phase(currentRef));
	else
		(**mIntfCurrent)(0,0) = currentRef.real();
}

void EMT::Ph1::CurrentSource::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mCurrentRef);
	modifiedAttributes.push_back(mRightVector);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::CurrentSource::mnaPreStep(Real time, Int timeStepCount) {
	updateState(time);
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::CurrentSource::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph1::CurrentSource::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaUpdateVoltage(**leftVector);
}

void EMT::Ph1::CurrentSource::mnaUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0,0) = 0;
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
}
