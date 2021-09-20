/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_Capacitor.h>

using namespace CPS;
using namespace CPS::DP::Ph1;

DP::Ph1::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	mEquivCurrent = { 0, 0 };
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);
	setTerminalNumber(2);

	addAttribute<Real>("C", &mCapacitance, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::Capacitor::clone(String name) {
	auto copy = Capacitor::make(name, mLogLevel);
	copy->setParameters(mCapacitance);
	return copy;
}

void DP::Ph1::Capacitor::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);

	mEquivCurrent = MatrixComp::Zero(mNumFreqs, 1);
	mEquivCond = MatrixComp::Zero(mNumFreqs, 1);
	mPrevVoltCoeff = MatrixComp::Zero(mNumFreqs, 1);
}

void DP::Ph1::Capacitor::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;
	Complex impedance = { 0, - 1. / (omega * mCapacitance) };
	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0,0) = mIntfVoltage(0,0) / impedance;

	mSLog->info("\nCapacitance [F]: {:s}"
				"\nImpedance [Ohm]: {:s}",
				Logger::realToString(mCapacitance),
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

void DP::Ph1::Capacitor::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	Real equivCondReal = 2.0 * mCapacitance / timeStep;
	Real prevVoltCoeffReal = 2.0 * mCapacitance / timeStep;

	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		Real equivCondImag = 2.*PI * mFrequencies(freq,0) * mCapacitance;
		mEquivCond(freq,0) = { equivCondReal, equivCondImag };
		Real prevVoltCoeffImag = - 2.*PI * mFrequencies(freq,0) * mCapacitance;
		mPrevVoltCoeff(freq,0) = { prevVoltCoeffReal, prevVoltCoeffImag };

		mEquivCurrent(freq,0) = -mIntfCurrent(0,freq) + -mPrevVoltCoeff(freq,0) * mIntfVoltage(0,freq);
		mIntfCurrent(0, freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
	}

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

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

void DP::Ph1::Capacitor::mnaInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	Real equivCondReal = 2.0 * mCapacitance / timeStep;
	Real prevVoltCoeffReal = 2.0 * mCapacitance / timeStep;

	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		Real equivCondImag = 2.*PI * mFrequencies(freq,0) * mCapacitance;
		mEquivCond(freq,0) = { equivCondReal, equivCondImag };
		Real prevVoltCoeffImag = - 2.*PI * mFrequencies(freq,0) * mCapacitance;
		mPrevVoltCoeff(freq,0) = { prevVoltCoeffReal, prevVoltCoeffImag };

		mEquivCurrent(freq,0) = -mIntfCurrent(0,freq) + -mPrevVoltCoeff(freq,0) * mIntfVoltage(0,freq);
		mIntfCurrent(0, freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
	}

	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
	mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::Capacitor::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
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
			mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			mEquivCond(freq,0).real(), mEquivCond(freq,0).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			mEquivCond(freq,0).real(), mEquivCond(freq,0).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			-mEquivCond(freq,0).real(), -mEquivCond(freq,0).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
			mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
			-mEquivCond(freq,0).real(), -mEquivCond(freq,0).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
		}
	}
}

void DP::Ph1::Capacitor::mnaApplySystemMatrixStampHarm(Matrix& systemMatrix, Int freqIdx) {
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
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
		mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
		mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
		-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
		mSLog->info("Add {:e}+j{:e} to system at ({:d},{:d})",
		-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
	}
}

void DP::Ph1::Capacitor::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		//mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
		//mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;
		mEquivCurrent(freq,0) = -mIntfCurrent(0,freq)
			+ -mPrevVoltCoeff(freq,0) * mIntfVoltage(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "MNA EquivCurrent {:f}+j{:f}",
			mEquivCurrent(freq,0).real(), mEquivCurrent(freq,0).imag());
		if (terminalNotGrounded(0))
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:f}+j{:f} to source vector at {:d}",
				mEquivCurrent(freq,0).real(), mEquivCurrent(freq,0).imag(), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:f}+j{:f} to source vector at {:d}",
				-mEquivCurrent(freq,0).real(), -mEquivCurrent(freq,0).imag(), matrixNodeIndex(1));
	}
}

void DP::Ph1::Capacitor::mnaApplyRightSideVectorStampHarm(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		//mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
		//mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;
		mEquivCurrent(freq,0) = -mIntfCurrent(0,freq)
			+ -mPrevVoltCoeff(freq,0) * mIntfVoltage(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0), 1, 0, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0), 1, 0, freq);
	}
}

void DP::Ph1::Capacitor::mnaApplyRightSideVectorStampHarm(Matrix& rightVector, Int freq) {
		//mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
		//mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;
		mEquivCurrent(freq,0) = -mIntfCurrent(0,freq)
			+ -mPrevVoltCoeff(freq,0) * mIntfVoltage(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0));
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0));
}

void DP::Ph1::Capacitor::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on C, but then we'd have to modify the system matrix anyway
	prevStepDependencies.push_back(this->attribute("i_intf"));
	prevStepDependencies.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("right_vector"));
}

void DP::Ph1::Capacitor::mnaPreStep(Real time, Int timeStepCount) {
	this->mnaApplyRightSideVectorStamp(this->mRightVector);
}

void DP::Ph1::Capacitor::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(this->attribute("v_intf"));
	modifiedAttributes.push_back(this->attribute("i_intf"));
}

void DP::Ph1::Capacitor::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(*leftVector);
	this->mnaUpdateCurrent(*leftVector);
}

void DP::Ph1::Capacitor::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
	mCapacitor.mnaApplyRightSideVectorStampHarm(mCapacitor.mRightVector);
}

void DP::Ph1::Capacitor::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
	for (UInt freq = 0; freq < mCapacitor.mNumFreqs; freq++)
		mCapacitor.mnaUpdateVoltageHarm(*mLeftVectors[freq], freq);
	mCapacitor.mnaUpdateCurrentHarm();
}

void DP::Ph1::Capacitor::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfVoltage(0,freq) = 0;
		if (terminalNotGrounded(1))
			mIntfVoltage(0,freq) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1), mNumFreqs, freq);
		if (terminalNotGrounded(0))
			mIntfVoltage(0,freq) = mIntfVoltage(0,freq) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:e}<{:e}", std::abs(mIntfVoltage(0,freq)), std::arg(mIntfVoltage(0,freq)));
	}
}

void DP::Ph1::Capacitor::mnaUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) {
	// v1 - v0
	mIntfVoltage(0,freqIdx) = 0;
	if (terminalNotGrounded(1))
		mIntfVoltage(0,freqIdx) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		mIntfVoltage(0,freqIdx) = mIntfVoltage(0,freqIdx) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString(mIntfVoltage(0,freqIdx)));
}

void DP::Ph1::Capacitor::mnaUpdateCurrent(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString(mIntfCurrent(0,freq)));
	}
}

void DP::Ph1::Capacitor::mnaUpdateCurrentHarm() {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		mIntfCurrent(0,freq) = mEquivCond(freq,0) * mIntfVoltage(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString(mIntfCurrent(0,freq)));
	}
}
