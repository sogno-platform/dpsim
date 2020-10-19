/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/SP/SP_Ph1_VoltageSource.h>

using namespace CPS;

SP::Ph1::VoltageSource::VoltageSource(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1, 1);
	mIntfCurrent = MatrixComp::Zero(1, 1);

	addAttribute<Complex>("V_ref", Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr SP::Ph1::VoltageSource::clone(String name) {
	auto copy = VoltageSource::make(name, mLogLevel);
	copy->setParameters(attribute<Complex>("V_ref")->get());
	return copy;
}

void SP::Ph1::VoltageSource::setParameters(Complex voltageRef) {
	attribute<Complex>("V_ref")->set(voltageRef);
	mParametersSet = true;
}

void SP::Ph1::VoltageSource::initializeFromNodesAndTerminals(Real frequency) {

	mVoltageRef = attribute<Complex>("V_ref");

	if (mVoltageRef->get() == Complex(0, 0))
		mVoltageRef->set(initialSingleVoltage(1) - initialSingleVoltage(0));

}

void SP::Ph1::VoltageSource::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	updateMatrixNodeIndices();
	mIntfVoltage(0, 0) = mVoltageRef->get();

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void SP::Ph1::VoltageSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), mVirtualNodes[0]->matrixNodeIndex(), Complex(-1, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(0), Complex(-1, 0));

	}
	if (terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), mVirtualNodes[0]->matrixNodeIndex(), Complex(1, 0));
		Math::addToMatrixElement(systemMatrix, mVirtualNodes[0]->matrixNodeIndex(), matrixNodeIndex(1), Complex(1, 0));
	}

}

void SP::Ph1::VoltageSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	Math::setVectorElement(rightVector, mVirtualNodes[0]->matrixNodeIndex(), mIntfVoltage(0, 0));

}

void SP::Ph1::VoltageSource::updateVoltage(Real time) {
	// can't we just do nothing??
	// TODO: remove updateVoltage
	mIntfVoltage(0, 0) = mVoltageRef->get();
}

void SP::Ph1::VoltageSource::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.updateVoltage(time);
	mVoltageSource.mnaApplyRightSideVectorStamp(mVoltageSource.mRightVector);
}

void SP::Ph1::VoltageSource::MnaPostStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.mnaUpdateCurrent(*mLeftVector);
}

void SP::Ph1::VoltageSource::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0, 0) = Math::complexFromVectorElement(leftVector, mVirtualNodes[0]->matrixNodeIndex());
}

void SP::Ph1::VoltageSource::daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) {
	/* new state vector definintion:
		state[0]=node0_voltage
		state[1]=node1_voltage
		....
		state[n]=noden_voltage
		state[n+1]=component0_voltage
		state[n+2]=component0_inductance (not yet implemented)
		...
		state[m-1]=componentm_voltage
		state[m]=componentm_inductance
	*/

	//int Pos1 = matrixNodeIndex(0);
	//int Pos2 = matrixNodeIndex(1);
	//int c_offset = off[0] + off[1]; //current offset for component
	//int n_offset_1 = c_offset + Pos1 + 1;// current offset for first nodal equation
	//int n_offset_2 = c_offset + Pos2 + 1;// current offset for second nodal equation
	//resid[c_offset] = (state[Pos2] - state[Pos1]) - state[c_offset]; // Voltage equation for Resistor
	////resid[++c_offset] = ; //TODO : add inductance equation
	//resid[n_offset_1] += std::real(current());
	//resid[n_offset_2] += std::real(current());
	//off[1] += 1;
}

Complex SP::Ph1::VoltageSource::daeInitialize() {
	mIntfVoltage(0, 0) = mVoltageRef->get();
	return mVoltageRef->get();
}
