/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_VoltageSource.h>

using namespace CPS;

EMT::Ph1::VoltageSource::VoltageSource(String uid, String name,	Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, true, true, logLevel),
	mVoltageRef(mAttributes->create<Complex>("V_ref")),
	mSrcFreq(mAttributes->create<Real>("f_src")) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);
}

void EMT::Ph1::VoltageSource::setParameters(Complex voltageRef, Real srcFreq) {
	**mVoltageRef = voltageRef;
	**mSrcFreq = srcFreq;

	mParametersSet = true;
}

SimPowerComp<Real>::Ptr EMT::Ph1::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(**mVoltageRef, **mSrcFreq);
	return copy;
}

void EMT::Ph1::VoltageSource::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	(**mIntfVoltage)(0,0) = Math::abs(**mVoltageRef) * cos(Math::phase(**mVoltageRef));
}

void EMT::Ph1::VoltageSource::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex(), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0), -1);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex(), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1), 1);
	}

	if (terminalNotGrounded(0)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex());
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0));
	}
	if (terminalNotGrounded(1)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex());
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1));
	}
}

void EMT::Ph1::VoltageSource::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(), (**mIntfVoltage)(0,0));
}

void EMT::Ph1::VoltageSource::updateVoltage(Real time) {
	Complex voltageRef = mVoltageRef->get();
	Real srcFreq = mSrcFreq->get();
	if (srcFreq > 0)
		(**mIntfVoltage)(0,0) = Math::abs(voltageRef) * cos(time * 2. * PI * srcFreq + Math::phase(voltageRef));
	else
		(**mIntfVoltage)(0,0) = voltageRef.real();
}

void EMT::Ph1::VoltageSource::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mVoltageRef);
	modifiedAttributes.push_back(mRightVector);
	modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph1::VoltageSource::mnaCompPreStep(Real time, Int timeStepCount) {
	updateVoltage(time);
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::VoltageSource::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::VoltageSource::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::VoltageSource::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0,0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
}
