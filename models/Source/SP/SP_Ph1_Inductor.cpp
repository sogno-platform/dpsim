/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_Inductor.h>

using namespace CPS;

SP::Ph1::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);
	setTerminalNumber(2);

	addAttribute<Real>("L", &mInductance, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr SP::Ph1::Inductor::clone(String name) {
	auto copy = Inductor::make(name, mLogLevel);
	copy->setParameters(mInductance);
	return copy;
}

void SP::Ph1::Inductor::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;
	mSusceptance = Complex(0, -1 / omega / mInductance);
	mIntfVoltage(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent = mSusceptance * mIntfVoltage;

	mSLog->info("\nInductance [H]: {:s}"
				"\nImpedance [Ohm]: {:s}",
				Logger::realToString(mInductance),
				Logger::complexToString(1./mSusceptance));
	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

// #### MNA section ####

void SP::Ph1::Inductor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	mSLog->info(
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString(mIntfVoltage(0, 0)),
		Logger::phasorToString(mIntfCurrent(0, 0)));
}

void SP::Ph1::Inductor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mSusceptance);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mSusceptance);
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mSusceptance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mSusceptance);

	}

	mSLog->info("-- Matrix Stamp ---");
	if (terminalNotGrounded(0))
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			mSusceptance.real(), mSusceptance.imag(), matrixNodeIndex(0), matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			mSusceptance.real(), mSusceptance.imag(), matrixNodeIndex(1), matrixNodeIndex(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			-mSusceptance.real(), -mSusceptance.imag(), matrixNodeIndex(0), matrixNodeIndex(1));
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			-mSusceptance.real(), -mSusceptance.imag(), matrixNodeIndex(1), matrixNodeIndex(0));
	}
}

void SP::Ph1::Inductor::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("i_intf"));
}

void SP::Ph1::Inductor::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(*leftVector);
	this->mnaUpdateCurrent(*leftVector);
}

void SP::Ph1::Inductor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mIntfVoltage = Matrix::Zero(3, 1);
	if (terminalNotGrounded(1)) {
		mIntfVoltage(0, 0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	}
	if (terminalNotGrounded(0)) {
		mIntfVoltage(0, 0) = mIntfVoltage(0, 0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	}
}

void SP::Ph1::Inductor::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent = mSusceptance*mIntfVoltage;
}


// #### Tear Methods ####
void SP::Ph1::Inductor::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, 1. / mSusceptance);
}


