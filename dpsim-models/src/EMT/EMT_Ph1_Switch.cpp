/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_Switch.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph1 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

///FIXME: Inconsistent naming in ph1 and p3 switches: (mIsClosed, mSwitchClosed)

EMT::Ph1::Switch::Switch(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, false, true, logLevel), Base::Ph1::Switch(mAttributes) {
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);
}

SimPowerComp<Real>::Ptr EMT::Ph1::Switch::clone(String name) {
	auto copy = Switch::make(name, mLogLevel);
	copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
	return copy;
}

void EMT::Ph1::Switch::initializeFromNodesAndTerminals(Real frequency) {

	Real resistance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;

	(**mIntfVoltage)(0,0) = (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
	(**mIntfCurrent)(0,0) = ((**mIntfVoltage)(0,0) / resistance);

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:s}"
		"\nCurrent: {:s}"
		"\nTerminal 0 voltage: {:s}"
		"\nTerminal 1 voltage: {:s}"
		"\n--- Initialization from powerflow finished ---",
		Logger::matrixToString(**mIntfVoltage),
		Logger::matrixToString(**mIntfCurrent),
		Logger::phasorToString(initialSingleVoltage(0)),
		Logger::phasorToString(initialSingleVoltage(1)));
}

void EMT::Ph1::Switch::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	updateMatrixNodeIndices();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

Bool EMT::Ph1::Switch::mnaIsClosed() { return **mIsClosed; }

void EMT::Ph1::Switch::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	Real conductance;

	conductance = (**mIsClosed) ?
		1./(**mClosedResistance) : 1./(**mOpenResistance);

	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), conductance);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), conductance);
	}
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -conductance);
	}
	SPDLOG_LOGGER_TRACE(mSLog, 
		"\nConductance matrix: {:s}",
		Logger::matrixToString(conductance));
}

void EMT::Ph1::Switch::mnaCompApplySwitchSystemMatrixStamp(Bool closed, SparseMatrixRow& systemMatrix, Int freqIdx) {
	Real conductance;

	conductance = (closed) ?
		1./(**mClosedResistance) : 1./(**mOpenResistance);

	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), conductance);

	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), conductance);
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -conductance);
	}

	SPDLOG_LOGGER_TRACE(mSLog, 
		"\nConductance matrix: {:s}",
		Logger::matrixToString(conductance));
}

void EMT::Ph1::Switch::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) { }

void EMT::Ph1::Switch::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::Switch::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::Switch::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	**mIntfVoltage = Matrix::Zero(1, 1);
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	}
}

void EMT::Ph1::Switch::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = (**mIsClosed) ?
		(**mIntfVoltage)(0, 0)/(**mClosedResistance):
		(**mIntfVoltage)(0, 0)/(**mOpenResistance);
}