/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_Inductor.h>

using namespace CPS;

DP::Ph1::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	mEquivCurrent = { 0, 0 };
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);
	setTerminalNumber(2);

	addAttribute<Real>("L", &mInductance, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::Inductor::clone(String name) {
	auto copy = Inductor::make(name, mLogLevel);
	copy->setParameters(mInductance);
	return copy;
}

void DP::Ph1::Inductor::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);

	mEquivCurrent = MatrixComp::Zero(mNumFreqs, 1);
	mEquivCond = MatrixComp::Zero(mNumFreqs, 1);
	mPrevCurrFac = MatrixComp::Zero(mNumFreqs, 1);
}

void DP::Ph1::Inductor::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2. * PI * frequency;
	Complex impedance = { 0, omega * mInductance };
	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	mSLog->info("\nInductance [H]: {:s}"
			 	"\nImpedance [Ohm]: {:s}",
				 Logger::realToString(mInductance),
				 Logger::complexToString(impedance));
	mSLog->info(
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

// #### MNA functions ####

void DP::Ph1::Inductor::initVars(Real timeStep) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		Real a = timeStep / (2. * mInductance);
		Real b = timeStep * 2.*PI * mFrequencies(freq,0) / 2.;

		Real equivCondReal = a / (1. + b * b);
		Real equivCondImag =  -a * b / (1. + b * b);
		mEquivCond(freq,0) = { equivCondReal, equivCondImag };
		Real preCurrFracReal = (1. - b * b) / (1. + b * b);
		Real preCurrFracImag =  (-2. * b) / (1. + b * b);
		mPrevCurrFac(freq,0) = { preCurrFracReal, preCurrFracImag };

		// In steady-state, these variables should not change
		mEquivCurrent(freq,0) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mPrevCurrFac(freq,0) * mIntfCurrent(0,freq);
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
	}
}

void DP::Ph1::Inductor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	initVars(timeStep);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	mSLog->info(
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\nEquiv. current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString(mIntfVoltage(0,0)),
		Logger::phasorToString(mIntfCurrent(0,0)),
		Logger::complexToString(mEquivCurrent(0,0)));
}

void DP::Ph1::Inductor::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	initVars(timeStep);

	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
	mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::Inductor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(1))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond(freq,0), mNumFreqs, freq);
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond(freq,0), mNumFreqs, freq);
		}

		mSLog->info("-- Stamp frequency {:d} ---", freq);
		if (terminalNotGrounded(0))
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(mEquivCond(freq,0)), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(mEquivCond(freq,0)), matrixNodeIndex(1), matrixNodeIndex(1));
		if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(-mEquivCond(freq,0)), matrixNodeIndex(0), matrixNodeIndex(1));
			mSLog->info("Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(-mEquivCond(freq,0)), matrixNodeIndex(1), matrixNodeIndex(0));
		}
	}
}

void DP::Ph1::Inductor::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freqIdx,0));
		if (terminalNotGrounded(1))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond(freqIdx,0));
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond(freqIdx,0));
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond(freqIdx,0));
		}

		mSLog->info("-- Stamp frequency {:d} ---", freqIdx);
		if (terminalNotGrounded(0))
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
		if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
			mSLog->info("Add {:f}+j{:f} to system at ({:d},{:d})",
				-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
		}
}

void DP::Ph1::Inductor::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		// Calculate equivalent current source for next time step
		mEquivCurrent(freq,0) =
			mEquivCond(freq,0) * mIntfVoltage(0,freq)
			+ mPrevCurrFac(freq,0) * mIntfCurrent(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "MNA EquivCurrent {:s}", Logger::complexToString(mEquivCurrent(freq,0)));
		if (terminalNotGrounded(0))
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
			Logger::complexToString(mEquivCurrent(freq,0)), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
			Logger::complexToString(-mEquivCurrent(freq,0)), matrixNodeIndex(1));
	}
}

void DP::Ph1::Inductor::mnaApplyRightSideVectorStampHarm(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		// Calculate equivalent current source for next time step
		mEquivCurrent(freq,0) =
			mEquivCond(freq,0) * mIntfVoltage(0,freq)
			+ mPrevCurrFac(freq,0) * mIntfCurrent(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0), 1, 0, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0), 1, 0, freq);
	}
}

void DP::Ph1::Inductor::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on L, but then we'd have to modify the system matrix anyway
	prevStepDependencies.push_back(this->attribute("v_intf"));
	prevStepDependencies.push_back(this->attribute("i_intf"));
	modifiedAttributes.push_back(this->attribute("right_vector"));
}

void DP::Ph1::Inductor::mnaPreStep(Real time, Int timeStepCount) {
	this->mnaApplyRightSideVectorStamp(this->mRightVector);
}

void DP::Ph1::Inductor::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("i_intf"));
}

void DP::Ph1::Inductor::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(*leftVector);
	this->mnaUpdateCurrent(*leftVector);
}

void DP::Ph1::Inductor::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
	mInductor.mnaApplyRightSideVectorStampHarm(mInductor.mRightVector);
}

void DP::Ph1::Inductor::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
	for (UInt freq = 0; freq < mInductor.mNumFreqs; freq++)
		mInductor.mnaUpdateVoltageHarm(*mLeftVectors[freq], freq);
	mInductor.mnaUpdateCurrentHarm();
}

void DP::Ph1::Inductor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfVoltage(0,freq) = 0;
		if (terminalNotGrounded(1))
			mIntfVoltage(0,freq) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1), mNumFreqs, freq);
		if (terminalNotGrounded(0))
			mIntfVoltage(0,freq) = mIntfVoltage(0,freq) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0,freq)));
	}
}

void DP::Ph1::Inductor::mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) {
	// v1 - v0
	mIntfVoltage(0,freqIdx) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,freqIdx) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,freqIdx) = mIntfVoltage(0,freqIdx) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0,freqIdx)));
}

void DP::Ph1::Inductor::mnaUpdateCurrent(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString(mIntfCurrent(0,freq)));
	}
}

void DP::Ph1::Inductor::mnaUpdateCurrentHarm() {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString(mIntfCurrent(0,freq)));
	}
}

// #### Tear Methods ####
void DP::Ph1::Inductor::mnaTearInitialize(Real omega, Real timeStep) {
	initVars(timeStep);
}

void DP::Ph1::Inductor::mnaTearApplyMatrixStamp(Matrix& tearMatrix) {
	Math::addToMatrixElement(tearMatrix, mTearIdx, mTearIdx, 1./mEquivCond(0,0));
}

void DP::Ph1::Inductor::mnaTearApplyVoltageStamp(Matrix& voltageVector) {
	mEquivCurrent(0,0) = mEquivCond(0,0) * mIntfVoltage(0,0) + mPrevCurrFac(0,0) * mIntfCurrent(0,0);
	Math::addToVectorElement(voltageVector, mTearIdx, mEquivCurrent(0,0) / mEquivCond(0,0));
}

void DP::Ph1::Inductor::mnaTearPostStep(Complex voltage, Complex current) {
	mIntfVoltage(0, 0) = voltage;
	mIntfCurrent(0, 0) = mEquivCond(0,0) * voltage + mEquivCurrent(0,0);

}
