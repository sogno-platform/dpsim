/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_Transformer.h>

using namespace CPS;

EMT::Ph3::Transformer::Transformer(String uid, String name,
	Logger::Level logLevel)
	: Base::Ph3::Transformer(mAttributes), 
	MNASimPowerComp<Real>(uid, name, true, true, logLevel),
	mPrimaryCurrent(mAttributes->create<Matrix>("primary_current")),
	mSecondaryCurrent(mAttributes->create<Matrix>("secondary_current")),
	mPrimaryLV(mAttributes->create<Matrix>("primary_voltage_LVside")),
	mSecondaryLV(mAttributes->create<Matrix>("secondary_voltage_LVside")) {
	
	//
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(2);

	//
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
	
	//
	mEquivCurrent = Matrix::Zero(3, 1);
	**mPrimaryCurrent = Matrix::Zero(3, 1);
	**mSecondaryCurrent = Matrix::Zero(3, 1);
	**mPrimaryLV = Matrix::Zero(3, 1);
	**mSecondaryLV = Matrix::Zero(3, 1);

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", this->type(), name);
}

void EMT::Ph3::Transformer::setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase,
	Matrix resistance, Matrix inductance) {

	Base::Ph3::Transformer::setParameters(nomVoltageEnd1, nomVoltageEnd2, ratioAbs, ratioPhase, resistance, inductance);

	SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1 = {} [V] Nominal Voltage End 2 = {} [V]", mNominalVoltageEnd1, mNominalVoltageEnd2);
    SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio = {} [ ] Phase Shift = {} [deg]", std::abs(**mRatio), std::arg(**mRatio));

	mParametersSet = true;
}

void EMT::Ph3::Transformer::initializeFromNodesAndTerminals(Real frequency) {
	// Component parameters are referred to higher voltage side.
	// Switch terminals to have terminal 0 at higher voltage side
	// if transformer is connected the other way around.
	if (Math::abs(**mRatio) < 1.) {
		**mRatio = 1. / **mRatio;
		std::shared_ptr<SimTerminal<Real>> tmp = mTerminals[0];
		mTerminals[0] = mTerminals[1];
		mTerminals[1] = tmp;
		Real tmpVolt = mNominalVoltageEnd1;
		mNominalVoltageEnd1 = mNominalVoltageEnd2;
		mNominalVoltageEnd2 = tmpVolt;
		SPDLOG_LOGGER_INFO(mSLog, "Switching terminals to have first terminal at higher voltage side. Updated parameters: ");
		SPDLOG_LOGGER_INFO(mSLog, "Nominal Voltage End 1 = {} [V] Nominal Voltage End 2 = {} [V]", mNominalVoltageEnd1, mNominalVoltageEnd2);
		SPDLOG_LOGGER_INFO(mSLog, "Tap Ratio = {} [ ] Phase Shift = {} [deg]", std::abs(**mRatio), std::arg(**mRatio));
	}

	// Static calculations from load flow data
	Real omega = 2. * PI * frequency;
	MatrixComp impedance = MatrixComp::Zero(3, 3);
	impedance <<
		Complex((mResistance)(0, 0), omega * (mInductance)(0, 0)), 0.0, 0.0,
		0.0, Complex((mResistance)(1, 1), omega * (mInductance)(1, 1)), 0.0,
		0.0, 0.0, Complex((mResistance)(2, 2), omega * (mInductance)(2, 2));

	SPDLOG_LOGGER_INFO(mSLog, "Resistance (referred to higher voltage side) = {} [Ohm]", Logger::matrixToString(mResistance));
	SPDLOG_LOGGER_INFO(mSLog, "Inductance (referred to higher voltage side) = {} [H]", Logger::matrixToString(mInductance));
	//SPDLOG_LOGGER_INFO(mSLog, "Impedance (referred to higher voltage side) = {} [Ohm]", Logger::matrixCompToString(impedance));

	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * (initialSingleVoltage(0) - initialSingleVoltage(1)* **mRatio);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;

	MatrixComp iInit = impedance.inverse() * vInitABC;
	**mIntfCurrent = iInit.real();
	**mIntfVoltage = vInitABC.real();

	//
	**mPrimaryLV = Math::singlePhaseVariableToThreePhase(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) * **mRatio).real();
	**mSecondaryLV = Math::singlePhaseVariableToThreePhase(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)).real();
	**mPrimaryCurrent = **mIntfCurrent;
	**mSecondaryCurrent = (iInit * **mRatio).real();

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(**mIntfVoltage),
		Logger::matrixToString(**mIntfCurrent),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)),
		Logger::phasorToString(RMS3PH_TO_PEAK1PH * initialSingleVoltage(1)));
	mSLog->flush();
}

void EMT::Ph3::Transformer::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	initVars(timeStep);
}

void EMT::Ph3::Transformer::initVars(Real timeStep) {
	// Assumption: symmetric R and L matrix
	Real a = timeStep * (mResistance)(0,0) / (2. * (mInductance)(0,0));
	Real b = timeStep / (2. * (mInductance)(0,0));

	mEquivCond = b / (1. + a);
	mPrevCurrFac = (1. - a ) / (1. + a);

	// Update internal state
	Matrix Voltage = Math::singlePhaseVariableToThreePhase(RMS3PH_TO_PEAK1PH * initialSingleVoltage(0)).real() -  **mPrimaryLV;
	mEquivCurrent = mEquivCond * Voltage + mPrevCurrFac * **mIntfCurrent;

	//
	//**mIntfCurrent = mEquivCond * Voltage + mEquivCurrent;

	//
	mSLog->info(
		"\n--- Initialization internal states ---"
		"\nInitial current {:s}"
		"\nEquiv. current {:s}"
		"\n--- Initialization internal states finished ---",
		Logger::matrixToString(**mIntfCurrent),
		Logger::matrixToString(mEquivCurrent));
	mSLog->flush();
}

void EMT::Ph3::Transformer::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), mEquivCond);
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), std::pow((**mRatio).real(),2) * mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), std::pow((**mRatio).real(),2) * mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), std::pow((**mRatio).real(),2) * mEquivCond);
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), - (**mRatio).real() * mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), - (**mRatio).real() * mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), - (**mRatio).real() * mEquivCond);

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), - (**mRatio).real() * mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), - (**mRatio).real() * mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), - (**mRatio).real() * mEquivCond);
	}

	//SPDLOG_LOGGER_INFO(mSLog,
	//	"\nEquivalent Conductance: {:s}",
	//	Logger::matrixToString(mEquivCond));
}

void EMT::Ph3::Transformer::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), -mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 1), -mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 2), -mEquivCurrent(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), (**mRatio).real() * mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 1), (**mRatio).real() * mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 2), (**mRatio).real() * mEquivCurrent(2, 0));
	}
}

void EMT::Ph3::Transformer::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	prevStepDependencies.push_back(mIntfVoltage);
	prevStepDependencies.push_back(mIntfCurrent);
	modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::Transformer::mnaCompPreStep(Real time, Int timeStepCount) {
	this->mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::Transformer::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::Transformer::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::Transformer::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	**mIntfVoltage = Matrix::Zero(3,1);
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) += Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		(**mIntfVoltage)(1, 0) += Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		(**mIntfVoltage)(2, 0) += Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	}
	if (terminalNotGrounded(1)) {
		(**mSecondaryLV)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
		(**mSecondaryLV)(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
		(**mSecondaryLV)(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
		**mIntfVoltage -= **mSecondaryLV;
		**mPrimaryLV = **mSecondaryLV * (**mRatio).real();
	}
}

void EMT::Ph3::Transformer::mnaCompUpdateCurrent(const Matrix& leftVector) {
	//
	Matrix Voltage = Matrix::Zero(3, 1);
	Voltage(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	Voltage(1,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
	Voltage(2,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	Voltage = Voltage - **mPrimaryLV;
	
	//
	**mIntfCurrent = mEquivCond * Voltage + mEquivCurrent;

	//
	**mPrimaryCurrent = **mIntfCurrent;
	**mSecondaryCurrent = **mIntfCurrent * (**mRatio).real();

	// Update internal state
	mEquivCurrent = mEquivCond * Voltage + mPrevCurrFac * **mIntfCurrent;	
}

