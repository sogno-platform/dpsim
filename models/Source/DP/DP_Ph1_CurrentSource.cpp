/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_CurrentSource.h>

using namespace CPS;

DP::Ph1::CurrentSource::CurrentSource(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel),
	mCurrentRef(CPS::Attribute<Complex>::create("I_ref", mAttributes)) {
	setTerminalNumber(2);
	**mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);
}

DP::Ph1::CurrentSource::CurrentSource(String name, Complex current, Logger::Level logLevel)
	: CurrentSource(name, logLevel) {
	setParameters(current);
}

void DP::Ph1::CurrentSource::setParameters(Complex current) {
	**mCurrentRef = current;
	mParametersSet = true;
}

SimPowerComp<Complex>::Ptr DP::Ph1::CurrentSource::clone(String name) {
	auto copy = CurrentSource::make(name, mLogLevel);
	copy->setParameters(**mCurrentRef);
	return copy;
}

void DP::Ph1::CurrentSource::initializeFromNodesAndTerminals(Real frequency) {

	(**mIntfVoltage)(0,0) = initialSingleVoltage(0) - initialSingleVoltage(1);
	(**mIntfCurrent)(0,0) = **mCurrentRef;

	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void DP::Ph1::CurrentSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	(**mIntfCurrent)(0,0) = **mCurrentRef;
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::CurrentSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mCurrentSource.mnaApplyRightSideVectorStamp(**mCurrentSource.mRightVector);
}

void DP::Ph1::CurrentSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	(**mIntfCurrent)(0,0) = **mCurrentRef;

	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, matrixNodeIndex(0), -(**mIntfCurrent)(0,0));
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, matrixNodeIndex(1), (**mIntfCurrent)(0,0));
}

void DP::Ph1::CurrentSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mCurrentSource.mnaUpdateVoltage(**mLeftVector);
}

void DP::Ph1::CurrentSource::mnaUpdateVoltage(const Matrix& leftVector) {
	(**mIntfVoltage)(0,0) = 0;
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
}
