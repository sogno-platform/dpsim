/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_Capacitor.h>

using namespace CPS;

EMT::Ph3::Capacitor::Capacitor(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, true, true, logLevel), Base::Ph3::Capacitor(mAttributes) {
	mPhaseType = PhaseType::ABC;
	setTerminalNumber(2);
	mEquivCurrent = Matrix::Zero(3, 1);
	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
}


SimPowerComp<Real>::Ptr EMT::Ph3::Capacitor::clone(String name) {
	auto copy = Capacitor::make(name, mLogLevel);
	copy->setParameters(**mCapacitance);
	return copy;
}
void EMT::Ph3::Capacitor::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;
	MatrixComp admittance = MatrixComp::Zero(3, 3);
	admittance <<
		Complex(0, omega * (**mCapacitance)(0, 0)), Complex(0, omega * (**mCapacitance)(0, 1)), Complex(0, omega * (**mCapacitance)(0, 2)),
		Complex(0, omega * (**mCapacitance)(1, 0)), Complex(0, omega * (**mCapacitance)(1, 1)), Complex(0, omega * (**mCapacitance)(1, 2)),
		Complex(0, omega * (**mCapacitance)(2, 0)), Complex(0, omega * (**mCapacitance)(2, 1)), Complex(0, omega * (**mCapacitance)(2, 2));

	MatrixComp vInitABC = Matrix::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) - RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	**mIntfVoltage = vInitABC.real();
	**mIntfCurrent = (admittance * vInitABC).real();

	SPDLOG_LOGGER_INFO(mSLog, "\nCapacitance [F]: {:s}"
				"\nAdmittance [S]: {:s}",
				Logger::matrixToString(**mCapacitance),
				Logger::matrixCompToString(admittance));
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

void EMT::Ph3::Capacitor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();
	mEquivCond = (2.0 * **mCapacitance) / timeStep;
	// Update internal state
	mEquivCurrent = - **mIntfCurrent + - mEquivCond * **mIntfVoltage;
}

void EMT::Ph3::Capacitor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), mEquivCond(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 1), mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 2), mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 0), mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 2), mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 0), mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 1), mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), mEquivCond(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 1), -mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 2), -mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 0), -mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), -mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 2), -mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 0), -mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 1), -mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), -mEquivCond(2, 2));

		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -mEquivCond(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 1), -mEquivCond(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 2), -mEquivCond(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 0), -mEquivCond(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), -mEquivCond(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 2), -mEquivCond(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 0), -mEquivCond(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 1), -mEquivCond(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), -mEquivCond(2, 2));
	}

	SPDLOG_LOGGER_INFO(mSLog,
			"\nEquivalent Conductance: {:s}",
			Logger::matrixToString(mEquivCond));

	mSLog->flush();
}

void EMT::Ph3::Capacitor::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	mEquivCurrent = -**mIntfCurrent + -mEquivCond * **mIntfVoltage;
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 1), mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 2), mEquivCurrent(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), -mEquivCurrent(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 1), -mEquivCurrent(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 2), -mEquivCurrent(2, 0));
	}
	SPDLOG_LOGGER_DEBUG(mSLog,
		"\nEquivalent Current: {:s}",
		Logger::matrixToString(mEquivCurrent));
}

void EMT::Ph3::Capacitor::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on C, but then we'd have to modify the system matrix anyway
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::Capacitor::mnaCompPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::Capacitor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::Capacitor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph3::Capacitor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	**mIntfVoltage = Matrix::Zero(3,1);
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
		(**mIntfVoltage)(1, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 1));
		(**mIntfVoltage)(2, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 2));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
		(**mIntfVoltage)(1, 0) = (**mIntfVoltage)(1, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
		(**mIntfVoltage)(2, 0) = (**mIntfVoltage)(2, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
	}
}

void EMT::Ph3::Capacitor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = mEquivCond * **mIntfVoltage + mEquivCurrent;
	SPDLOG_LOGGER_DEBUG(mSLog,
		"\nCurrent: {:s}",
		Logger::matrixToString(**mIntfCurrent)
	);
}

