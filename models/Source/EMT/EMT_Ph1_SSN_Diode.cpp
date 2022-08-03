/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph1_SSN_Diode.h>

using namespace CPS;

EMT::Ph1::SSN::Diode::Diode(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::Single;
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(1, 1);
	**mIntfCurrent = Matrix::Zero(1, 1);
}

void EMT::Ph1::SSN::Diode::setParameters(Real I_S, Real V_T){
	mI_S = I_S;
	mV_T = V_T;
}


SimPowerComp<Real>::Ptr EMT::Ph1::SSN::Diode::clone(String name) {
	auto copy = Diode::make(name, mLogLevel);
	copy->setParameters(mI_S, mV_T);
	return copy;
}

void EMT::Ph1::SSN::Diode::initializeFromNodesAndTerminals(Real frequency) {

	// IntfVoltage initialization for each phase
	MatrixComp vInitABC = Matrix::Zero(1, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) - RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
	(**mIntfVoltage)(0, 0) = vInitABC(0, 0).real();

	///FIXME: 	Initialization should include solving the system once to obtain the actual values solving the
	//			system for the 0th time step. As of now abnormal current values for the 0th time step
	//			are to be expected.
	
	(**mIntfCurrent)(0, 0) = mI_S*(expf((**mIntfVoltage)(0, 0)/mV_T)-1.);

	mSLog->info(
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
}

void EMT::Ph1::SSN::Diode::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);

	updateMatrixNodeIndices();

	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	**SSN_Function_Result = Matrix::Zero(leftVector->get().rows(), 1);
	SSNcalculateFunctionResult();
    ssnUpdateJacobian();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

void EMT::Ph1::SSN::Diode::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), Jacobian(0, 0));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), Jacobian(0, 0));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -Jacobian(0, 0));


		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -Jacobian(0, 0));
	}

	mSLog->info(
		"\nJacobian: {:s}",
		Logger::matrixToString(Jacobian));
}

void EMT::Ph1::SSN::Diode::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph1::SSN::Diode::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	//mnaUpdateVoltage(**leftVector);
	//mnaUpdateCurrent(**leftVector);
}


void EMT::Ph1::SSN::Diode::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	**mIntfVoltage = Matrix::Zero(3,1);
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	}
	mSLog->debug(
		"\nVoltage: {:s}",
		Logger::matrixToString(**mIntfVoltage)
	);
	mSLog->flush();
}

void EMT::Ph1::SSN::Diode::mnaUpdateCurrent(const Matrix& leftVector) {
	(**mIntfCurrent)(0, 0) = mI_S*(expf((**mIntfVoltage)(0, 0)/mV_T)-1.);
	mSLog->debug(
		"\nCurrent: {:s}",
		Logger::matrixToString(**mIntfCurrent)
	);
	mSLog->flush();
}

void EMT::Ph1::SSN::Diode::ssnUpdate(const Matrix& leftVector){
    //Update phase voltages
	if (terminalNotGrounded(1)) {
		(**mIntfVoltage)(0, 0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
	}
	if (terminalNotGrounded(0)) {
		(**mIntfVoltage)(0, 0) = (**mIntfVoltage)(0, 0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
	}

	SSNcalculateFunctionResult();

    ssnUpdateJacobian();

}


void EMT::Ph1::SSN::Diode::SSNcalculateFunctionResult(){

	(**mIntfCurrent)(0, 0) = mI_S*(expf((**mIntfVoltage)(0, 0)/mV_T)-1.);

	if(terminalNotGrounded(1)){
		Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(1, 0), (**mIntfCurrent)(0, 0));
	}
	if(terminalNotGrounded(0)){
		Math::setVectorElement(**SSN_Function_Result, matrixNodeIndex(0, 0), -(**mIntfCurrent)(0, 0));
	}
}


void EMT::Ph1::SSN::Diode::ssnUpdateJacobian(){
	Jacobian(0, 0) = (mI_S/mV_T)*expf((**mIntfVoltage)(0, 0)/mV_T);
}