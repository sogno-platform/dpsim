/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_Resistor.h>

using namespace CPS;

EMT::Ph1::Resistor::Resistor(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, false, true, logLevel), Base::Ph1::Resistor(mAttributes) {
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);
}

SimPowerComp<Real>::Ptr EMT::Ph1::Resistor::clone(String name) {
	auto copy = Resistor::make(name, mLogLevel);
	copy->setParameters(**mResistance);
	return copy;
}

void EMT::Ph1::Resistor::initializeFromNodesAndTerminals(Real frequency) {

	(**mIntfVoltage)(0,0) = (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
	(**mIntfCurrent)(0,0) = (**mIntfVoltage)(0,0) / **mResistance;

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- Initialization from powerflow ---"
		"\nVoltage across: {:f}"
		"\nCurrent: {:f}"
		"\nTerminal 0 voltage: {:f}"
		"\nTerminal 1 voltage: {:f}"
		"\n--- Initialization from powerflow finished ---",
		(**mIntfVoltage)(0,0),
		(**mIntfCurrent)(0,0),
		initialSingleVoltage(0).real(),
		initialSingleVoltage(1).real());
}

void EMT::Ph1::Resistor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	**mRightVector = Matrix::Zero(0, 0);
}

void EMT::Ph1::Resistor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	Real conductance = 1. / **mResistance;
	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0)  &&  terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -conductance);
	}

	if (terminalNotGrounded(0))
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", conductance, matrixNodeIndex(0), matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", conductance, matrixNodeIndex(1), matrixNodeIndex(1));
	if ( terminalNotGrounded(0)  &&  terminalNotGrounded(1) ) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -conductance, matrixNodeIndex(0), matrixNodeIndex(1));
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -conductance, matrixNodeIndex(1), matrixNodeIndex(0));
	}
}

void EMT::Ph1::Resistor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::Resistor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::Resistor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	(**mIntfVoltage)(0,0) = 0;
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::Resistor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0,0) = (**mIntfVoltage)(0,0) / **mResistance;
}

// #### DAE functions ####

void EMT::Ph1::Resistor::daeInitialize(double time, double state[], double dstate_dt[],
	double absoluteTolerances[], double stateVarTypes[], int& offset) {
	updateMatrixNodeIndices();
	mSLog->info(
		"\n--- daeInitialize ---"
		"\nno variable was added by the resistor '{:s}' to the state vector"
		"\n--- daeInitialize finished ---",
		this->name());
	mSLog->flush();
}

void EMT::Ph1::Resistor::daeResidual(double sim_time,
	const double state[], const double dstate_dt[],
	double resid[], std::vector<int>& off) {
	// state[Pos2] = voltage of node matrixNodeIndex(1)
	// state[Pos1] = voltage of node matrixNodeIndex(0)
	// resid[Pos1] = nodal current equation of node matrixNodeIndex(1) --> add resistor current
	// resid[Pos2] = nodal current equation of node matrixNodeIndex(0) --> substract resistor current

	int node1 = matrixNodeIndex(0);
    int node2 = matrixNodeIndex(1);

	double voltageResistor = 0.0;
	if (terminalNotGrounded(0))
		voltageResistor -= state[node1];
	if (terminalNotGrounded(1))
		voltageResistor +=state[node2];
	
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		resid[node1] -= (state[node2] - state[node1]) / **mResistance;
		resid[node2] += (state[node2] - state[node1]) / **mResistance;
	} else if (terminalNotGrounded(0)) {
		resid[node1] -= state[node1] / **mResistance;
	} else if (terminalNotGrounded(1)) {
		resid[node2] += state[node2] / **mResistance;
	}
}

void EMT::Ph1::Resistor::daeJacobian(double current_time, const double state[], 
			const double dstate_dt[], SUNMatrix jacobian, double cj, std::vector<int>& off) {

	int node_pos0 = matrixNodeIndex(0);
    int node_pos1 = matrixNodeIndex(1);
	int c_offset = off[0] + off[1]; //current offset for component

	if (terminalNotGrounded(1))
		SM_ELEMENT_D(jacobian, node_pos1, node_pos1) += 1.0 / **mResistance;

	if (terminalNotGrounded(0))
		SM_ELEMENT_D(jacobian, node_pos0, node_pos0) += 1.0 / **mResistance;

	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		SM_ELEMENT_D(jacobian, node_pos1, node_pos0) += -1.0 / **mResistance;
		SM_ELEMENT_D(jacobian, node_pos0, node_pos1) += -1.0 / **mResistance;
	}
}

void EMT::Ph1::Resistor::daePostStep(double Nexttime, const double state[], const double dstate_dt[], int& offset) {
	int node1 = matrixNodeIndex(0);
    int node2 = matrixNodeIndex(1);
	(**mIntfVoltage)(0,0) = 0.0;
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0,0) -= state[node1];
	}
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0,0) += state[node2];
	}
	(**mIntfCurrent)(0,0) = (**mIntfVoltage)(0,0) / **mResistance;
}