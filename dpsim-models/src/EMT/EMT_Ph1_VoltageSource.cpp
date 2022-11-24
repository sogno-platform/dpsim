/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph1_VoltageSource.h>

using namespace CPS;

EMT::Ph1::VoltageSource::VoltageSource(String uid, String name,	Logger::Level logLevel)
	: MNASimPowerComp<Real>(uid, name, true, true, logLevel),
	mVoltageRef(mAttributes->create<Complex>("V_ref")),
	mSrcFreq(mAttributes->create<Real>("f_src")) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);
}

void EMT::Ph1::VoltageSource::setParameters(Complex voltageRef, Real srcFreq) {
	**mVoltageRef = voltageRef;
	**mSrcFreq = srcFreq;

	mParametersSet = true;
}

SimPowerComp<Real>::Ptr EMT::Ph1::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(**mVoltageRef, **mSrcFreq);
	return copy;
}

void EMT::Ph1::VoltageSource::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();
	(**mIntfVoltage)(0,0) = Math::abs(**mVoltageRef) * cos(Math::phase(**mVoltageRef));
}

void EMT::Ph1::VoltageSource::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex(), -1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0), -1);
	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex(), 1);
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1), 1);
	}

	if (terminalNotGrounded(0)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex());
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", -1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0));
	}
	if (terminalNotGrounded(1)) {
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex());
		SPDLOG_LOGGER_INFO(mSLog, "Add {:f} to system at ({:d},{:d})", 1., mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1));
	}
}

void EMT::Ph1::VoltageSource::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(), (**mIntfVoltage)(0,0));
}

void EMT::Ph1::VoltageSource::updateVoltage(Real time) {
	Complex voltageRef = mVoltageRef->get();
	Real srcFreq = mSrcFreq->get();
	if (srcFreq > 0)
		(**mIntfVoltage)(0,0) = Math::abs(voltageRef) * cos(time * 2.*PI*srcFreq + Math::phase(voltageRef));
	else
		(**mIntfVoltage)(0,0) = voltageRef.real();
}

void EMT::Ph1::VoltageSource::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mVoltageRef);
	modifiedAttributes.push_back(mRightVector);
	modifiedAttributes.push_back(mIntfVoltage);
}

void EMT::Ph1::VoltageSource::mnaCompPreStep(Real time, Int timeStepCount) {
	updateVoltage(time);
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph1::VoltageSource::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph1::VoltageSource::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateCurrent(**leftVector);
}

void EMT::Ph1::VoltageSource::mnaCompUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0,0) = Math::realFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
}

// #### DAE functions ####
void EMT::Ph1::VoltageSource::setInitialComplexIntfCurrent(Complex initCurrent) {
	//set initial current 
    (**mIntfCurrent)(0, 0) = initCurrent.real();
	
	// Calculate initial derivative of current = -omega*Imag(Complex_voltage)
	Real omega = 2 * PI * attribute<Real>("f_src")->get();

	mIntfDerCurrent = Matrix::Zero(1, 1);
	mIntfDerCurrent(0,0) = -omega * initCurrent.imag();
}

void EMT::Ph1::VoltageSource::daeInitialize(double time, double state[], double dstate_dt[], 
	double absoluteTolerances[], double stateVarTypes[], int& offset) {
	// offset: number of component in state, dstate_dt
	// state[offset] = current through voltage source flowing into node matrixNodeIndex(1)
	// dstate_dt[offset] = derivative of current through voltage source

	updateMatrixNodeIndices();

	this->updateVoltage(time);

	state[offset] = (**mIntfCurrent)(0,0);
	dstate_dt[offset] = mIntfDerCurrent(0,0);

	//set state variable as algebraic variable
	stateVarTypes[offset]  = 0.0;

	//set absolute tolerance 
	absoluteTolerances[offset] = mAbsTolerance;

	mSLog->info(
		"\n--- daeInitialize ---"
		"\nAdded current through VoltageSource '{:s}' to state vector, initial value  = {:f}A"
		"\nAdded derivative of current through VoltageSource '{:s}' to derivative state vector, initial value= {:f}"
		"\nState variable set as algebraic"
		"\nAbsolute tolerance={:f}"
		"\n--- daeInitialize finished ---",
		this->name(), state[offset],
		this->name(), dstate_dt[offset],
		absoluteTolerances[offset]
	);
	mSLog->flush();
	offset++;
}

void EMT::Ph1::VoltageSource::daeResidual(double sim_time, 
	const double state[], const double dstate_dt[], 
	double resid[], std::vector<int>& off) {
	// state[c_offset] = current through VoltageSource flowing into the voltage source
	// dstate_dt[offset] = derivative of current through voltage source
	// resid[c_offset] = v2-v1-v_s = voltage(state[Pos2]) - voltage(state[Pos1]) - mIntfVoltage(0,0)
	// resid[Pos1] = nodal current equation of node matrixNodeIndex(0)
	// resid[Pos2] = nodal current equation of node matrixNodeIndex(1)

	updateVoltage(sim_time);

	int Pos1 = matrixNodeIndex(0);
    int Pos2 = matrixNodeIndex(1);
	int c_offset = off[0] + off[1]; //current offset for component

	resid[c_offset] = -(**mIntfVoltage)(0,0);
	if (terminalNotGrounded(0)) {
		resid[c_offset] -= state[Pos1];	
		resid[Pos1] -= state[c_offset];
	}
	if (terminalNotGrounded(1)) {
		resid[c_offset] += state[Pos2];
		resid[Pos2] += state[c_offset];
	}
	off[1] += 1;
}

void EMT::Ph1::VoltageSource::daeJacobian(double current_time, const double state[], 
			const double dstate_dt[], SUNMatrix jacobian, double cj, std::vector<int>& off) {

	int node_pos0 = matrixNodeIndex(0);
    int node_pos1 = matrixNodeIndex(1);
	int c_offset = off[0] + off[1]; //current offset for component

	if (terminalNotGrounded(1)) {
		SM_ELEMENT_D(jacobian, c_offset, node_pos1) += 1.0;
		SM_ELEMENT_D(jacobian, node_pos1, c_offset) += 1.0;
	}

	if (terminalNotGrounded(0)) {
		SM_ELEMENT_D(jacobian, c_offset, node_pos0) += -1.0;
		SM_ELEMENT_D(jacobian, node_pos0, c_offset) += -1.0;
	}

	off[1] += 1;
}

void EMT::Ph1::VoltageSource::daePostStep(double Nexttime, const double state[], 
	const double dstate_dt[], int& offset) {

	(**mIntfCurrent)(0,0) = state[offset];
	mIntfDerCurrent(0,0) = dstate_dt[offset];
	(**mIntfVoltage)(0,0) = 0.0;
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0,0) += state[matrixNodeIndex(1)];
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0,0) -= state[matrixNodeIndex(0)];
	}
	offset++;
}

