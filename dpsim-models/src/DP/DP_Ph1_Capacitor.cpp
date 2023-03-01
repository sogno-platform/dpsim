/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_Capacitor.h>

using namespace CPS;
using namespace CPS::DP::Ph1;

DP::Ph1::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Complex>(uid, name, true, true, logLevel), Base::Ph1::Capacitor(mAttributes) {
	mEquivCurrent = { 0, 0 };
	**mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);
	setTerminalNumber(2);
}

SimPowerComp<Complex>::Ptr DP::Ph1::Capacitor::clone(String name) {
	auto copy = Capacitor::make(name, mLogLevel);
	copy->setParameters(**mCapacitance);
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
	Complex impedance = { 0, - 1. / (omega * **mCapacitance) };
	(**mIntfVoltage)(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	(**mIntfCurrent)(0,0) = (**mIntfVoltage)(0,0) / impedance;

	SPDLOG_LOGGER_INFO(mSLog, "\nCapacitance [F]: {:s}"
				"\nImpedance [Ohm]: {:s}",
				Logger::realToString(**mCapacitance),
				Logger::complexToString(impedance));
	SPDLOG_LOGGER_INFO(mSLog,
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
	mSLog->flush();
}

void DP::Ph1::Capacitor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();

	Real equivCondReal = 2.0 * **mCapacitance / timeStep;
	Real prevVoltCoeffReal = 2.0 * **mCapacitance / timeStep;

	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		Real equivCondImag = 2.*PI * mFrequencies(freq,0) * **mCapacitance;
		mEquivCond(freq,0) = { equivCondReal, equivCondImag };
		Real prevVoltCoeffImag = - 2.*PI * mFrequencies(freq,0) * **mCapacitance;
		mPrevVoltCoeff(freq,0) = { prevVoltCoeffReal, prevVoltCoeffImag };

		mEquivCurrent(freq,0) = -(**mIntfCurrent)(0,freq) + -mPrevVoltCoeff(freq,0) * (**mIntfVoltage)(0,freq);
		(**mIntfCurrent)(0, freq) = mEquivCond(freq,0) * (**mIntfVoltage)(0,freq) + mEquivCurrent(freq,0);
	}

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\nEquiv. current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::complexToString(mEquivCurrent(0,0)));
	mSLog->flush();
}

void DP::Ph1::Capacitor::mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
		updateMatrixNodeIndices();

	Real equivCondReal = 2.0 * **mCapacitance / timeStep;
	Real prevVoltCoeffReal = 2.0 * **mCapacitance / timeStep;

	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		Real equivCondImag = 2.*PI * mFrequencies(freq,0) * **mCapacitance;
		mEquivCond(freq,0) = { equivCondReal, equivCondImag };
		Real prevVoltCoeffImag = - 2.*PI * mFrequencies(freq,0) * **mCapacitance;
		mPrevVoltCoeff(freq,0) = { prevVoltCoeffReal, prevVoltCoeffImag };

		mEquivCurrent(freq,0) = -(**mIntfCurrent)(0,freq) + -mPrevVoltCoeff(freq,0) * (**mIntfVoltage)(0,freq);
		(**mIntfCurrent)(0, freq) = mEquivCond(freq,0) * (**mIntfVoltage)(0,freq) + mEquivCurrent(freq,0);
	}

	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
	**mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::Capacitor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(1))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond(freq,0), mNumFreqs, freq);
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond(freq,0), mNumFreqs, freq);
		}

		SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freq);
		if (terminalNotGrounded(0))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			mEquivCond(freq,0).real(), mEquivCond(freq,0).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			mEquivCond(freq,0).real(), mEquivCond(freq,0).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			-mEquivCond(freq,0).real(), -mEquivCond(freq,0).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
			SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
			-mEquivCond(freq,0).real(), -mEquivCond(freq,0).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
		}
	}
}

void DP::Ph1::Capacitor::mnaCompApplySystemMatrixStampHarm(SparseMatrixRow& systemMatrix, Int freqIdx) {
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freqIdx,0));
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond(freqIdx,0));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond(freqIdx,0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond(freqIdx,0));
	}

	SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freqIdx);
	if (terminalNotGrounded(0))
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
		mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
		mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
		-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
		SPDLOG_LOGGER_INFO(mSLog, "Add {:e}+j{:e} to system at ({:d},{:d})",
		-mEquivCond(freqIdx,0).real(), -mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
	}
}

void DP::Ph1::Capacitor::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		//mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
		//mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;
		mEquivCurrent(freq,0) = -(**mIntfCurrent)(0,freq)
			+ -mPrevVoltCoeff(freq,0) * (**mIntfVoltage)(0,freq);

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

void DP::Ph1::Capacitor::mnaCompApplyRightSideVectorStampHarm(Matrix& rightVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		//mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
		//mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;
		mEquivCurrent(freq,0) = -(**mIntfCurrent)(0,freq)
			+ -mPrevVoltCoeff(freq,0) * (**mIntfVoltage)(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0), 1, 0, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0), 1, 0, freq);
	}
}

void DP::Ph1::Capacitor::mnaCompApplyRightSideVectorStampHarm(Matrix& rightVector, Int freq) {
		//mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
		//mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;
		mEquivCurrent(freq,0) = -(**mIntfCurrent)(0,freq)
			+ -mPrevVoltCoeff(freq,0) * (**mIntfVoltage)(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0));
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent(freq,0));
}

void DP::Ph1::Capacitor::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on C, but then we'd have to modify the system matrix anyway
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void DP::Ph1::Capacitor::mnaCompPreStep(Real time, Int timeStepCount) {
	this->mnaApplyRightSideVectorStamp(**this->mRightVector);
}

void DP::Ph1::Capacitor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::Capacitor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	this->mnaUpdateVoltage(**leftVector);
	this->mnaUpdateCurrent(**leftVector);
}

void DP::Ph1::Capacitor::MnaPreStepHarm::execute(Real time, Int timeStepCount) {
	mCapacitor.mnaCompApplyRightSideVectorStampHarm(**mCapacitor.mRightVector);
}

void DP::Ph1::Capacitor::MnaPostStepHarm::execute(Real time, Int timeStepCount) {
	for (UInt freq = 0; freq < mCapacitor.mNumFreqs; freq++)
		mCapacitor.mnaCompUpdateVoltageHarm(**mLeftVectors[freq], freq);
	mCapacitor.mnaCompUpdateCurrentHarm();
}

void DP::Ph1::Capacitor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfVoltage)(0,freq) = 0;
		if (terminalNotGrounded(1))
			(**mIntfVoltage)(0,freq) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1), mNumFreqs, freq);
		if (terminalNotGrounded(0))
			(**mIntfVoltage)(0,freq) = (**mIntfVoltage)(0,freq) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:e}<{:e}", std::abs((**mIntfVoltage)(0,freq)), std::arg((**mIntfVoltage)(0,freq)));
	}
}

void DP::Ph1::Capacitor::mnaCompUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) {
	// v1 - v0
	(**mIntfVoltage)(0,freqIdx) = 0;
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,freqIdx) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,freqIdx) = (**mIntfVoltage)(0,freqIdx) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString((**mIntfVoltage)(0,freqIdx)));
}

void DP::Ph1::Capacitor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfCurrent)(0,freq) = mEquivCond(freq,0) * (**mIntfVoltage)(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString((**mIntfCurrent)(0,freq)));
	}
}

void DP::Ph1::Capacitor::mnaCompUpdateCurrentHarm() {
	for (UInt freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfCurrent)(0,freq) = mEquivCond(freq,0) * (**mIntfVoltage)(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString((**mIntfCurrent)(0,freq)));
	}
}
