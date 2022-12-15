/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_Inductor.h>

using namespace CPS;

EMT::Ph1::Inductor::Inductor(String uid, String name, Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, true, true, logLevel), 
	Base::Ph1::Inductor(mAttributes),
	mIntfDerCurrent(Attribute<Matrix>::create("di_intf", mAttributes)) {

	mEquivCurrent = 0;
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);

	// initialize dae specific attributes
	**mIntfDerCurrent = Matrix::Zero(1,1);

	//
	setTerminalNumber(2);
}

SimPowerComp<Real>::Ptr EMT::Ph1::Inductor::clone(String name) {
	auto copy = Inductor::make(name, mLogLevel);
	copy->setParameters(**mInductance);
	return copy;
}

void EMT::Ph1::Inductor::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;
	Complex impedance = { 0, omega * **mInductance };
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

void EMT::Ph1::Inductor::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();

	mEquivCond = timeStep / (2.0 * **mInductance);
	// Update internal state
	mEquivCurrent = mEquivCond * (**mIntfVoltage)(0,0) + (**mIntfCurrent)(0,0);
}

void EMT::Ph1::Inductor::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mEquivCond);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mEquivCond);
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -mEquivCond);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -mEquivCond);
	}
}

void EMT::Ph1::Inductor::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	// Update internal state
	mEquivCurrent = mEquivCond * (**mIntfVoltage)(0,0) + (**mIntfCurrent)(0,0);
	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, matrixNodeIndex(0), mEquivCurrent);
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, matrixNodeIndex(1), -mEquivCurrent);
}

void EMT::Ph1::Inductor::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on L, but then we'd have to modify the system matrix anyway
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfCurrent);
	prevStepDependencies.push_back(mIntfVoltage);
}

void EMT::Ph1::Inductor::mnaCompPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::Inductor::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::Inductor::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::Inductor::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	(**mIntfVoltage)(0,0) = 0;
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::Inductor::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0,0) = mEquivCond * (**mIntfVoltage)(0,0) + mEquivCurrent;
}

// #### DAE Section ####

void EMT::Ph1::Inductor::daeInitialize(double time, double state[], double dstate_dt[], 
	double absoluteTolerances[], double stateVarTypes[], int &offset) {
	// state[offset] = current through inductor
	// dstate_dt[offset] = inductor current derivative
	updateMatrixNodeIndices();

	// set init values of states
	state[offset] = (**mIntfCurrent)(0,0);	
	dstate_dt[offset] = (**mIntfVoltage)(0,0) / **mInductance;		
	(**mIntfDerCurrent)(0,0) = dstate_dt[offset];
	
	//set state variable as differential variable
	stateVarTypes[offset]  = 1.0;

	//set absolute tolerance
	absoluteTolerances[offset] = mAbsTolerance;

	mSLog->info(
		"\n--- daeInitialize ---"
		"\nAdded current through inductor '{:s}' to state vector, initial value = {:f}A"
		"\nAdded derivative of current inductor '{:s}' to derivative state vector, initial value = {:f}"
		"\nState variable set as differential"
		"\nAbsolute tolerance={:f}"
		"\n--- daeInitialize finished ---",
		this->name(), state[offset], 
		this->name(), dstate_dt[offset],
		absoluteTolerances[offset]
	);
	mSLog->flush();

	offset++;
}

void EMT::Ph1::Inductor::daeResidual(double sim_time, 
	const double state[], const double dstate_dt[], 
	double resid[], std::vector<int>& off) {
	// state[c_offset] = current through inductor, flowing into node matrixNodeIndex(0)
	// dstate_dt[c_offset] = inductor current derivative

	// current offset for component
	int c_offset = off[0] + off[1]; 

	resid[c_offset] = - **mInductance * dstate_dt[c_offset];
	if (terminalNotGrounded(0)) {
		resid[c_offset] -= state[matrixNodeIndex(0)];
		resid[matrixNodeIndex(0)] -= state[c_offset];
	}
	if (terminalNotGrounded(1)) {
		resid[c_offset] += state[matrixNodeIndex(1)];
		resid[matrixNodeIndex(1)] += state[c_offset];
	}

	off[1] += 1;
}

void EMT::Ph1::Inductor::daeJacobian(double current_time, const double state[], 
	const double dstate_dt[], SUNMatrix jacobian, double cj, std::vector<int>& off) {

	// current offset for component			
	int c_offset = off[0] + off[1]; 

	SM_ELEMENT_D(jacobian, c_offset, c_offset) -= cj * **mInductance;
	if (terminalNotGrounded(1)) {
		SM_ELEMENT_D(jacobian, c_offset, matrixNodeIndex(1)) += 1.0;
		SM_ELEMENT_D(jacobian, matrixNodeIndex(1), c_offset) += 1.0;
	}

	if (terminalNotGrounded(0)) {
		SM_ELEMENT_D(jacobian, c_offset, matrixNodeIndex(0)) += -1.0;
		SM_ELEMENT_D(jacobian, matrixNodeIndex(0), c_offset) += -1.0;
	}

	off[1] += 1;
}

void EMT::Ph1::Inductor::daePostStep(double Nexttime, const double state[], 
	const double dstate_dt[], int& offset) {
		
	(**mIntfVoltage)(0,0) = 0.0;
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) -= state[matrixNodeIndex(0)];
	if (terminalNotGrounded(1)) 
		(**mIntfVoltage)(0,0) += state[matrixNodeIndex(1)];

	(**mIntfCurrent)(0,0) = state[offset];
	(**mIntfDerCurrent)(0,0) = dstate_dt[offset];

	offset++;
}
