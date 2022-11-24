/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_Capacitor.h>

using namespace CPS;

EMT::Ph1::Capacitor::Capacitor(String uid, String name,	Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, true, true, logLevel), 
	Base::Ph1::Capacitor(mAttributes) 
	mIntfDerVoltage(Attribute<Matrix>::create("dv_intf", mAttributes)), {
	mEquivCurrent = 0;
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);
	setTerminalNumber(2);

	// initialize dae specific attributes
	**mIntfDerVoltage = Matrix::Zero(1,1);
}

SimPowerComp<Real>::Ptr EMT::Ph1::Capacitor::clone(String name) {
	auto copy = Capacitor::make(name, mLogLevel);
	copy->setParameters(**mCapacitance);
	return copy;
}

void EMT::Ph1::Capacitor::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;
	Complex impedance = { 0, - 1. / (omega * **mCapacitance) };
	(**mIntfVoltage)(0,0) = (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
	(**mIntfCurrent)(0,0) = ((initialSingleVoltage(1) - initialSingleVoltage(0)) / impedance).real();

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

void EMT::Ph1::Capacitor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();

	mEquivCond = (2.0 * **mCapacitance) / timeStep;
	// Update internal state
	mEquivCurrent = -(**mIntfCurrent)(0,0) + -mEquivCond * (**mIntfVoltage)(0,0);
}

void EMT::Ph1::Capacitor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond);
	}
}

void EMT::Ph1::Capacitor::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	mEquivCurrent = -(**mIntfCurrent)(0,0) + -mEquivCond * (**mIntfVoltage)(0,0);
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent);
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent);
}

void EMT::Ph1::Capacitor::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on C, but then we'd have to modify the system matrix anyway
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
}


void EMT::Ph1::Capacitor::mnaCompPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::Capacitor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::Capacitor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::Capacitor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	(**mIntfVoltage)(0,0) = 0;
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::Capacitor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0,0) = mEquivCond * (**mIntfVoltage)(0,0) + mEquivCurrent;
}

// #### DAE functions ####

void EMT::Ph1::Capacitor::daeInitialize(double time, double state[], double dstate_dt[],
	double absoluteTolerances[], double stateVarTypes[], int &offset) {
	
	updateMatrixNodeIndices();

	mSLog->info(
		"\n--- daeInitialize ---"
		"\nNo state variables are needed"
		"\n--- daeInitialize finished ---"
	);
	mSLog->flush();
}

void EMT::Ph1::Capacitor::daeResidual(double sim_time, 
	const double state[], const double dstate_dt[], 
	double resid[], std::vector<int>& off) {

	int node_pos0 = matrixNodeIndex(0);
    int node_pos1 = matrixNodeIndex(1);
	int c_offset = off[0] + off[1]; //current offset for component

	// add currents to node equations
	if (terminalNotGrounded(0)) {
		resid[node_pos0] -= **mCapacitance * dstate_dt[node_pos0];
	}
	if (terminalNotGrounded(1)) {
		resid[node_pos1] += **mCapacitance * dstate_dt[node_pos1];
	}
}

void EMT::Ph1::Capacitor::daeJacobian(double current_time, const double state[], const double dstate_dt[], 
			SUNMatrix jacobian, double cj, std::vector<int>& off) {

	int node_pos0 = matrixNodeIndex(0);
    int node_pos1 = matrixNodeIndex(1);

	if (terminalNotGrounded(1))
		SM_ELEMENT_D(jacobian, node_pos1, node_pos1) += cj * **mCapacitance;

	if (terminalNotGrounded(0))
		SM_ELEMENT_D(jacobian, node_pos0, node_pos0) += cj * **mCapacitance;
	
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		SM_ELEMENT_D(jacobian, node_pos1, node_pos0) += - cj * **mCapacitance;
		SM_ELEMENT_D(jacobian, node_pos0, node_pos1) += - cj * **mCapacitance;
	}
}

void EMT::Ph1::Capacitor::daePostStep(double Nexttime, const double state[], 
	const double dstate_dt[], int& offset) {
	
	int node_pos0 = matrixNodeIndex(0);
    int node_pos1 = matrixNodeIndex(1);

	(**mIntfCurrent)(0,0) = 0.0;
	(**mIntfVoltage)(0,0) = 0.0;
	(**mIntfDerVoltage)(0,0) = 0.0;

	if (terminalNotGrounded(1)) {
		(**mIntfCurrent)(0,0) += **mCapacitance * dstate_dt[node_pos1];
		(**mIntfVoltage)(0,0) += state[node_pos1];
		(**mIntfDerVoltage)(0,0) += dstate_dt[node_pos1]; 
	}
	if (terminalNotGrounded(0)) {
		(**mIntfCurrent)(0,0) -= **mCapacitance * dstate_dt[node_pos0];
		(**mIntfVoltage)(0,0) -= state[node_pos0];
		(**mIntfDerVoltage)(0,0) -= dstate_dt[node_pos0]; 
	}
}


