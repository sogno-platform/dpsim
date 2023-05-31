/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph1_Transformer.h>

using namespace CPS;

DP::Ph1::Transformer::Transformer(String uid, String name, Logger::Level logLevel)
	: Base::Ph1::Transformer(mAttributes), 
      MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
	  mPrimaryCurrent(mAttributes->create<Complex>("primary_current")),
	  mSecondaryCurrent(mAttributes->create<Complex>("secondary_current")),
	  mPrimaryLV(mAttributes->create<Complex>("primary_voltage_LVside")),
	  mSecondaryLV(mAttributes->create<Complex>("secondary_voltage_LVside")) {
	
	//
	setTerminalNumber(2);

	//
	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
	
	//
	**mIntfVoltage = MatrixComp::Zero(1,1);
	**mIntfCurrent = MatrixComp::Zero(1,1);
}

void DP::Ph1::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	Base::Ph1::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);

	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
	SPDLOG_LOGGER_INFO(mSLog, "Resistance={} [Ohm] Inductance={} [Ohm] (referred to primary side)", **mResistance, **mInductance);
    SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio={} [ ] Phase Shift={} [deg]", std::abs(**mRatio), std::arg(**mRatio));

	mParametersSet = true;
}

void DP::Ph1::Transformer::initialize(Matrix frequencies) {
	SimPowerComp<Complex>::initialize(frequencies);

	mEquivCurrent = MatrixComp::Zero(mNumFreqs, 1);
	mEquivCond = MatrixComp::Zero(mNumFreqs, 1);
	mPrevCurrFac = MatrixComp::Zero(mNumFreqs, 1);
}

void DP::Ph1::Transformer::initializeFromNodesAndTerminals(Real frequency) {

	// Component parameters are referred to higher voltage side.
	// Switch terminals to have terminal 0 at higher voltage side
	// if transformer is connected the other way around.
	if (Math::abs(**mRatio) < 1.) {
		**mRatio = 1. / **mRatio;
		std::shared_ptr<SimTerminal<Complex>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
		Real tmpVolt = **mNominalVoltageEnd1;
		**mNominalVoltageEnd1 = **mNominalVoltageEnd2;
		**mNominalVoltageEnd2 = tmpVolt;
		SPDLOG_LOGGER_INFO(mSLog, "Switching terminals to have first terminal at higher voltage side. Updated parameters: ");
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1 = {} [V] Nominal Voltage End 2 = {} [V]", **mNominalVoltageEnd1, **mNominalVoltageEnd2);
		SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio = {} [ ] Phase Shift = {} [deg]", std::abs(**mRatio), std::arg(**mRatio));
	}
		   
	// Static calculations from load flow data
	Real omega = 2. * PI * frequency;
	Complex mImpedance  = Complex(**mResistance, omega * **mInductance);
	(**mIntfVoltage)(0,0) = initialSingleVoltage(0) - initialSingleVoltage(1);
	(**mIntfCurrent)(0,0) = (initialSingleVoltage(0) - initialSingleVoltage(1) * **mRatio) / mImpedance;

	//
	**mPrimaryLV = initialSingleVoltage(1) * **mRatio;
	**mSecondaryLV = initialSingleVoltage(1);
	**mPrimaryCurrent = (**mIntfCurrent)(0, 0);
	**mSecondaryCurrent = (**mIntfCurrent)(0, 0) * **mRatio;

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\nVirtual Node 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
	mSLog->flush();

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nHV side Current: {:s} (= {:s})"
		"\nLow side Current: {:s}"
		"\nTerminal 0 voltage (HV side voltage): {:s}"
		"\nTerminal 1 voltage (LV side voltage): {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::phasorToString((**mIntfVoltage)(0, 0)),
		Logger::phasorToString(**mPrimaryCurrent),
		Logger::complexToString(**mPrimaryCurrent),
		Logger::phasorToString(**mSecondaryCurrent),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
	mSLog->flush();
}

void DP::Ph1::Transformer::initVars(Real timeStep) {
	for (Int freq = 0; freq < mNumFreqs; freq++) {
		Real a = timeStep / (2. * **mInductance);
		Real b = (timeStep * 2.*PI * mFrequencies(freq,0)) / 2.;
		Real c = (1 + a * **mResistance) * (1 + a * **mResistance) + b * b;

		Real equivCondReal = ( a + (a * a * **mResistance) )  / c;
		Real equivCondImag =  -(a * b) / c;
		mEquivCond(freq,0) = { equivCondReal, equivCondImag };
		Real preCurrFracReal = (( 1. - b * b ) - pow((a * **mResistance),2)) / c;
		Real preCurrFracImag =  (-2. * b) / c;
		mPrevCurrFac(freq,0) = { preCurrFracReal, preCurrFracImag };

		//
		mEquivCurrent(freq,0) = mEquivCond(freq,0) * (initialSingleVoltage(0) - initialSingleVoltage(1) * **mRatio) + 
								mPrevCurrFac(freq,0) * (**mIntfCurrent)(0, 0);
		(**mIntfCurrent)(0,freq) = mEquivCond(freq,0) * (initialSingleVoltage(0) - initialSingleVoltage(1) * **mRatio) + mEquivCurrent(freq,0);
	}
}

void DP::Ph1::Transformer::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	initVars(timeStep);

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\nEquiv. current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::phasorToString((**mIntfVoltage)(0,0)),
		Logger::phasorToString((**mIntfCurrent)(0,0)),
		Logger::complexToString(mEquivCurrent(0,0)));
}

void DP::Ph1::Transformer::mnaCompInitializeHarm(Real omega, Real timeStep, std::vector<Attribute<Matrix>::Ptr> leftVectors) {
	updateMatrixNodeIndices();

	initVars(timeStep);

	mMnaTasks.push_back(std::make_shared<MnaPreStepHarm>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStepHarm>(*this, leftVectors));
	**mRightVector = Matrix::Zero(leftVectors[0]->get().rows(), mNumFreqs);
}

void DP::Ph1::Transformer::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	for (Int freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(1))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), std::pow(**mRatio,2) * mEquivCond(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -**mRatio * mEquivCond(freq,0), mNumFreqs, freq);
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -**mRatio * mEquivCond(freq,0), mNumFreqs, freq);
		}

		SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freq);
		if (terminalNotGrounded(0))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(mEquivCond(freq,0)), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(std::pow(**mRatio,2) * mEquivCond(freq,0)), matrixNodeIndex(1), matrixNodeIndex(1));
		if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(- **mRatio * mEquivCond(freq,0)), matrixNodeIndex(0), matrixNodeIndex(1));
			SPDLOG_LOGGER_INFO(mSLog, "Add {:s} to system at ({:d},{:d})",
				Logger::complexToString(- **mRatio * mEquivCond(freq,0)), matrixNodeIndex(1), matrixNodeIndex(0));
		}
	}
}

void DP::Ph1::Transformer::mnaCompApplySystemMatrixStampHarm(SparseMatrixRow& systemMatrix, Int freqIdx) {
		if (terminalNotGrounded(0))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond(freqIdx,0));
		if (terminalNotGrounded(1))
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), std::pow(**mRatio,2) * mEquivCond(freqIdx,0));
		if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -**mRatio * mEquivCond(freqIdx,0));
			Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -**mRatio * mEquivCond(freqIdx,0));
		}

		SPDLOG_LOGGER_INFO(mSLog, "-- Stamp frequency {:d} ---", freqIdx);
		if (terminalNotGrounded(0))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:f}+j{:f} to system at ({:d},{:d})",
				mEquivCond(freqIdx,0).real(), mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			SPDLOG_LOGGER_INFO(mSLog, "Add {:f}+j{:f} to system at ({:d},{:d})",
				**mRatio * **mRatio * mEquivCond(freqIdx,0).real(), std::pow(**mRatio,2) * mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(1));
		if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
			SPDLOG_LOGGER_INFO(mSLog, "Add {:f}+j{:f} to system at ({:d},{:d})",
				- **mRatio * mEquivCond(freqIdx,0).real(), - **mRatio * mEquivCond(freqIdx,0).imag(), matrixNodeIndex(0), matrixNodeIndex(1));
			SPDLOG_LOGGER_INFO(mSLog, "Add {:f}+j{:f} to system at ({:d},{:d})",
				- **mRatio * mEquivCond(freqIdx,0).real(), - **mRatio * mEquivCond(freqIdx,0).imag(), matrixNodeIndex(1), matrixNodeIndex(0));
		}
} 

void DP::Ph1::Transformer::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	for (Int freq = 0; freq < mNumFreqs; freq++) {
		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), -mEquivCurrent(freq,0), mNumFreqs, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), **mRatio * mEquivCurrent(freq,0), mNumFreqs, freq);

		SPDLOG_LOGGER_DEBUG(mSLog, "MNA EquivCurrent {:s}", Logger::complexToString(mEquivCurrent(freq,0)));
		if (terminalNotGrounded(0))
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
			Logger::complexToString(mEquivCurrent(freq,0)), matrixNodeIndex(0));
		if (terminalNotGrounded(1))
			SPDLOG_LOGGER_DEBUG(mSLog, "Add {:s} to source vector at {:d}",
			Logger::complexToString(- **mRatio * mEquivCurrent(freq,0)), matrixNodeIndex(1));
	}
}

 void DP::Ph1::Transformer::mnaCompApplyRightSideVectorStampHarm(Matrix& rightVector) { 
	for (Int freq = 0; freq < mNumFreqs; freq++) {
		// Calculate equivalent current source for next time step
		mEquivCurrent(freq,0) =
			mEquivCond(freq,0) * (initialSingleVoltage(0) - **mPrimaryLV) + 
			mPrevCurrFac(freq,0) * (**mIntfCurrent)(0,freq);

		if (terminalNotGrounded(0))
			Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent(freq,0), 1, 0, freq);
		if (terminalNotGrounded(1))
			Math::setVectorElement(rightVector, matrixNodeIndex(1), - **mRatio * mEquivCurrent(freq,0), 1, 0, freq);
	}
}

void DP::Ph1::Transformer::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfVoltage);
	prevStepDependencies.push_back(mIntfCurrent);
}

void DP::Ph1::Transformer::mnaCompPreStep(Real time, Int timeStepCount) {
	this->mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::Transformer::MnaPreStepHarm::execute(Real time, Int timeStepCount) { 
	mTransformer.mnaCompApplyRightSideVectorStampHarm(**mTransformer.mRightVector);
}

void DP::Ph1::Transformer::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph1::Transformer::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph1::Transformer::MnaPostStepHarm::execute(Real time, Int timeStepCount) { 
	for (Int freq = 0; freq < mTransformer.mNumFreqs; freq++)
		mTransformer.mnaCompUpdateVoltageHarm(**mLeftVectors[freq], freq);
	mTransformer.mnaCompUpdateCurrentHarm();
}

void DP::Ph1::Transformer::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v0 - v1
	for (Int freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfVoltage)(0,freq) = 0;
		if (terminalNotGrounded(0))
			(**mIntfVoltage)(0,freq) += Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq);
		if (terminalNotGrounded(1)) {
			// TODO: add other frequencies
			**mSecondaryLV = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1), mNumFreqs, freq);
			**mPrimaryLV = **mSecondaryLV * **mRatio;
			(**mIntfVoltage)(0,freq) -= **mSecondaryLV;
		}
			
		SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString((**mIntfVoltage)(0,freq)));
	}
}

void DP::Ph1::Transformer::mnaCompUpdateVoltageHarm(const Matrix& leftVector, Int freqIdx) { 
	// CHECK
	// v0 - v1
	(**mIntfVoltage)(0,freqIdx) = 0;
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,freqIdx) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,freqIdx) = (**mIntfVoltage)(0,freqIdx) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));

	SPDLOG_LOGGER_DEBUG(mSLog, "Voltage {:s}", Logger::phasorToString((**mIntfVoltage)(0,freqIdx)));
}

void DP::Ph1::Transformer::mnaCompUpdateCurrent(const Matrix& leftVector) {
	for (Int freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfCurrent)(0,freq) = mEquivCond(freq,0) * (Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq) - **mPrimaryLV) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString((**mIntfCurrent)(0,freq)));


		// TODO: add other frequencies
		**mPrimaryCurrent = (**mIntfCurrent)(0, 0);
		**mSecondaryCurrent = (**mIntfCurrent)(0, 0) * **mRatio;

		// Calculate equivalent current source for next time step
		mEquivCurrent(freq,0) =
			mEquivCond(freq,0) * (Math::complexFromVectorElement(leftVector, matrixNodeIndex(0), mNumFreqs, freq) - **mPrimaryLV) 
			+  mPrevCurrFac(freq,0) * (**mIntfCurrent)(0,freq);
		}
}

void DP::Ph1::Transformer::mnaCompUpdateCurrentHarm() {
	for (Int freq = 0; freq < mNumFreqs; freq++) {
		(**mIntfCurrent)(0,freq) = mEquivCond(freq,0) * (**mIntfVoltage)(0,freq) + mEquivCurrent(freq,0);
		SPDLOG_LOGGER_DEBUG(mSLog, "Current {:s}", Logger::phasorToString((**mIntfCurrent)(0,freq)));
	}
}