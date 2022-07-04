/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph1_SSN_Full_Serial_RLC.h>
#include <iostream>

using namespace CPS;

EMT::Ph1::SSN::Full_Serial_RLC::Full_Serial_RLC(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {

	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);
	setTerminalNumber(2);

	///FIXME: Initialization should happen in the base class declaring the attribute. However, this base class is currently not an AttributeList...
	mInductance = CPS::Attribute<Real>::create("L", mAttributes);
    mCapacitance = CPS::Attribute<Real>::create("C", mAttributes);
    mResistance = CPS::Attribute<Real>::create("R", mAttributes);
}

void EMT::Ph1::SSN::Full_Serial_RLC::setParameters(Real resistance, Real inductance, Real capacitance)
{
    **mInductance = inductance;
    **mCapacitance = capacitance;
    **mResistance = resistance;
    mParametersSet = true;
}

SimPowerComp<Real>::Ptr EMT::Ph1::SSN::Full_Serial_RLC::clone(String name) {
	auto copy = Full_Serial_RLC::make(name, mLogLevel);
	copy->setParameters(**mResistance, **mInductance, **mCapacitance);
	return copy;
}

void EMT::Ph1::SSN::Full_Serial_RLC::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;
	Complex impedance = { **mResistance, omega * **mInductance - 1./(omega * **mCapacitance)};
	(**mIntfVoltage)(0,0) = (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
	(**mIntfCurrent)(0,0) = ((**mIntfVoltage)(0,0) / impedance).real();

	mSLog->info(
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

void EMT::Ph1::SSN::Full_Serial_RLC::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);

	updateMatrixNodeIndices();

	State = Matrix::Zero(2, 1);
    yHistory =  Matrix::Zero(1, 1);

    Dufour_A_k_hat = Matrix::Zero(2, 2);
	Dufour_B_k_hat = Matrix::Zero(2, 1);
    Dufour_B_k_n_hat = Matrix::Zero(2, 1);
	Dufour_W_k_n = Matrix::Zero(1, 1);
    Dufour_C_k_n = Matrix(1, 2);

    Dufour_A_k_hat(0, 0) = 1.-  ((  2.*(timeStep*timeStep))/(4.* (**mInductance)* (**mCapacitance) +
                                    2. * timeStep * (**mCapacitance) * (**mResistance) +
                                    timeStep*timeStep)
                                );
    Dufour_A_k_hat(0, 1) = (timeStep/(2. * (**mCapacitance))) * (1. +     ((4. * (**mInductance) * (**mCapacitance) - 
                                    2. * timeStep * (**mResistance) * (**mCapacitance) -
                                    (timeStep*timeStep))/
                                    (
                                    4. * (**mInductance) * (**mCapacitance) + 
                                    2. * timeStep * (**mResistance) * (**mCapacitance) +
                                    (timeStep*timeStep))
                                    ));
    Dufour_A_k_hat(1, 0) = -1. *((4. * (**mCapacitance) * timeStep)/(
                                4. * (**mInductance) * (**mCapacitance) +
                                2. * timeStep* (**mCapacitance)* (**mResistance) +
                                (timeStep*timeStep)
                                ));
    Dufour_A_k_hat(1, 1) =    (4. * (**mInductance) * (**mCapacitance) - 
                                2. * timeStep * (**mResistance) * (**mCapacitance) -                                 
                                (timeStep*timeStep))/                                                                                                       
                                (
                                4. * (**mInductance) * (**mCapacitance) + 
                                2. * timeStep * (**mResistance) * (**mCapacitance) +
                                (timeStep*timeStep)
                                );

    Dufour_B_k_hat(0, 0) =  (timeStep*timeStep)/   
                            (4. * (**mInductance) * (**mCapacitance) +
                            2. * timeStep * (**mCapacitance) * (**mResistance) +
                            (timeStep * timeStep));

    Dufour_B_k_hat(1, 0) =  (timeStep * 2. * (**mCapacitance))/   
                            (4. * (**mInductance) * (**mCapacitance) +
                            2. * timeStep * (**mCapacitance) * (**mResistance) +
                            (timeStep * timeStep));

    Dufour_B_k_n_hat = Dufour_B_k_hat;

    Dufour_C_k_n(0, 1) = 1.;

    Dufour_W_k_n = Dufour_C_k_n * Dufour_B_k_n_hat;

	///FIXME:	mIntfCurrent is state 2 and is potentially directly initialized by other initialization methodes (e.g. FromNodesAndTerminals).
	///			State 1, which is Voltage over the capacitor, is not directly initialized and has to be calculated from the states. This is why
	///			the current must be reset as it would be altered as well. However, the old value of state one (time step "-1") is unknown or "zero"
	///			in this case, so calculation of state 1 would always assume zero as the past value of state 1 and also takes mIntfCurrent
	///			for the calculation ->State 1 is ahead of state 2 by one step, but also always (wrongly?) assumes past state 1 to be zero.
	///			How to handle properly?
	State(1, 0) = (**mIntfCurrent)(0, 0);
	ssnUpdateState();
	State(1, 0) = (**mIntfCurrent)(0, 0);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), Dufour_W_k_n(0,0));
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), Dufour_W_k_n(0,0));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -Dufour_W_k_n(0,0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -Dufour_W_k_n(0,0));
	}
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Update internal state
	yHistory = Dufour_C_k_n * (Dufour_A_k_hat * State +  Dufour_B_k_hat * **mIntfVoltage);

	if (terminalNotGrounded(0))
		Math::setVectorElement(rightVector, matrixNodeIndex(0), yHistory(0,0));
	if (terminalNotGrounded(1))
		Math::setVectorElement(rightVector, matrixNodeIndex(1), -yHistory(0,0));
}

void EMT::Ph1::SSN::Full_Serial_RLC::MnaPreStep::execute(Real time, Int timeStepCount) {
	mFull_Serial_RLC.mnaApplyRightSideVectorStamp(**mFull_Serial_RLC.mRightVector);
}

void EMT::Ph1::SSN::Full_Serial_RLC::MnaPostStep::execute(Real time, Int timeStepCount) {
	mFull_Serial_RLC.mnaUpdateVoltage(**mLeftVector);
	mFull_Serial_RLC.mnaUpdateCurrent(**mLeftVector);
	mFull_Serial_RLC.ssnUpdateState();
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	Dufour_u_n_t = **mIntfVoltage;
	(**mIntfVoltage)(0,0) = 0;
	if (terminalNotGrounded(1))
		(**mIntfVoltage)(0,0) = Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0))
		(**mIntfVoltage)(0,0) = (**mIntfVoltage)(0,0) - Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
}

void EMT::Ph1::SSN::Full_Serial_RLC::mnaUpdateCurrent(const Matrix& leftVector) {

    **mIntfCurrent = yHistory + Dufour_W_k_n * **mIntfVoltage; 
}

void EMT::Ph1::SSN::Full_Serial_RLC::ssnUpdateState()
{
	State = Dufour_A_k_hat * State + Dufour_B_k_hat * Dufour_u_n_t + Dufour_B_k_n_hat * **mIntfVoltage;
}