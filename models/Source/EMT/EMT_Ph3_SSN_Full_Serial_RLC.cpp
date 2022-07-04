/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SSN_Full_Serial_RLC.h>

using namespace CPS;

EMT::Ph3::SSN::Full_Serial_RLC::Full_Serial_RLC(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	mPhaseType = PhaseType::ABC;

	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
	setTerminalNumber(2);

	///FIXME: Initialization should happen in the base class declaring the attribute. However, this base class is currently not an AttributeList...
	mInductance = CPS::Attribute<Matrix>::create("L", mAttributes);
    mCapacitance = CPS::Attribute<Matrix>::create("C", mAttributes);
    mResistance = CPS::Attribute<Matrix>::create("R", mAttributes);
}

void EMT::Ph3::SSN::Full_Serial_RLC::setParameters(Matrix resistance, Matrix inductance, Matrix capacitance)
{
    **mInductance = inductance;
    **mCapacitance = capacitance;
    **mResistance = resistance;
    mParametersSet = true;
}

SimPowerComp<Real>::Ptr EMT::Ph3::SSN::Full_Serial_RLC::clone(String name) {
	auto copy = Full_Serial_RLC::make(name, mLogLevel);
	copy->setParameters(**mResistance, **mInductance, **mCapacitance);
	return copy;
}

void EMT::Ph3::SSN::Full_Serial_RLC::initializeFromNodesAndTerminals(Real frequency) {

	Real omega = 2 * PI * frequency;

	MatrixComp impedance = MatrixComp::Zero(3, 3);

    impedance <<
		Complex((**mResistance)(0, 0), omega * (**mInductance)(0, 0) - 1./(omega * (**mCapacitance)(0, 0))),
		Complex((**mResistance)(0, 1), omega * (**mInductance)(0, 1) - 1./(omega * (**mCapacitance)(0, 1))), 
		Complex((**mResistance)(0, 2), omega * (**mInductance)(0, 2) - 1./(omega * (**mCapacitance)(0, 2))), 
		Complex((**mResistance)(1, 0), omega * (**mInductance)(1, 0) - 1./(omega * (**mCapacitance)(1, 0))), 
		Complex((**mResistance)(1, 1), omega * (**mInductance)(1, 1) - 1./(omega * (**mCapacitance)(1, 1))), 
		Complex((**mResistance)(1, 2), omega * (**mInductance)(1, 2) - 1./(omega * (**mCapacitance)(1, 2))), 
		Complex((**mResistance)(2, 0), omega * (**mInductance)(2, 0) - 1./(omega * (**mCapacitance)(2, 0))), 
		Complex((**mResistance)(2, 1), omega * (**mInductance)(2, 1) - 1./(omega * (**mCapacitance)(2, 1))),
		Complex((**mResistance)(2, 2), omega * (**mInductance)(2, 2) - 1./(omega * (**mCapacitance)(2, 2)));


	MatrixComp vInitABC = Matrix::Zero(3, 1);
	vInitABC(0, 0) = RMS3PH_TO_PEAK1PH * initialSingleVoltage(1) - RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	**mIntfVoltage = vInitABC.real();
	MatrixComp admittance = impedance.inverse();
	**mIntfCurrent = (admittance * vInitABC).real();

	///FIXME: mIntfCurrent gets initialized to a vector of NaNs!
	**mIntfCurrent = Matrix::Zero(3,3);


	mSLog->info("\nImpedance [Ohm]: {:s}",
				Logger::matrixCompToString(impedance));
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

void EMT::Ph3::SSN::Full_Serial_RLC::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);

	updateMatrixNodeIndices();

	State = Matrix::Zero(6, 1);
    yHistory =  Matrix::Zero(3, 1);

    //Fill Dufour_A_k_hat
    //State Equation one, phases A,B,C: top left submatrix
    Dufour_A_k_hat(0, 0) = 1.-   ((2.*(timeStep*timeStep))/(4.* (**mInductance)(0, 0)* (**mCapacitance)(0, 0) +
                                                            2. * timeStep * (**mCapacitance)(0, 0) * (**mResistance)(0, 0) +
                                                            timeStep*timeStep)
                                    );
    Dufour_A_k_hat(1, 1) = 1.-   ((2.*(timeStep*timeStep))/(4.* (**mInductance)(1, 1)* (**mCapacitance)(1, 1) +
                                                            2. * timeStep * (**mCapacitance)(1, 1) * (**mResistance)(1, 1) +
                                                            timeStep*timeStep)
                                    );
    Dufour_A_k_hat(2, 2) = 1.-   ((2.*(timeStep*timeStep))/(4.* (**mInductance)(2, 2)*(**mCapacitance)(2, 2) +
                                                            2. * timeStep * (**mCapacitance)(2, 2) * (**mResistance)(2, 2) +
                                                            timeStep*timeStep)
                                    );
    //State Equation one, phases A,B,C: top right submatrix
    Dufour_A_k_hat(0, 3) = (timeStep/(2. * (**mCapacitance)(0, 0))) * (1. +     ((4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) - 
                                                                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) -
                                                                                (timeStep*timeStep))/
                                                                            (
                                                                                4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) + 
                                                                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) +
                                                                                (timeStep*timeStep))
                                                                                )
                                                                        );
    Dufour_A_k_hat(1, 4) = (timeStep/(2. * (**mCapacitance)(1, 1))) * (1. +     ((4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) - 
                                                                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) -
                                                                                (timeStep*timeStep))/
                                                                            (
                                                                                4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) + 
                                                                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) +
                                                                                (timeStep*timeStep))
                                                                                )
                                                                        );
    Dufour_A_k_hat(2, 5) = (timeStep/(2. * (**mCapacitance)(2, 2))) * (1. +     ((4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) - 
                                                                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) -
                                                                                (timeStep*timeStep))/
                                                                            (
                                                                                4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) + 
                                                                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) +
                                                                                (timeStep*timeStep))
                                                                                )
                                                                        );
    //State Equation two, phases A,B,C: bottom left submatrix
    Dufour_A_k_hat(3, 0) = -  ((4. * (**mCapacitance)(0, 0) * timeStep)/(
                                4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) +
                                2. * timeStep* (**mCapacitance)(0, 0)* (**mResistance)(0, 0) +
                                (timeStep*timeStep)
                                ));
    Dufour_A_k_hat(4, 1) = -  ((4. * (**mCapacitance)(1, 1) * timeStep)/(
                                4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) +
                                2. * timeStep* (**mCapacitance)(1, 1)* (**mResistance)(1, 1) +
                                (timeStep*timeStep)
                                ));
    Dufour_A_k_hat(5, 2) = -  ((4. * (**mCapacitance)(2, 2) * timeStep)/(
                                4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) +
                                2. * timeStep* (**mCapacitance)(2, 2)* (**mResistance)(2, 2) +
                                (timeStep*timeStep)
                                ));
    //State Equation two, phases A,B,C: bottom right submatrix
    Dufour_A_k_hat(3, 3) =    (4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) - 
                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) -                                 
                                (timeStep*timeStep))/                                                                                                       
                                (
                                4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) + 
                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) +
                                (timeStep*timeStep)
                                );
    Dufour_A_k_hat(4, 4) =    (4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) - 
                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) -                                 
                                (timeStep*timeStep))/                                                                                                       
                                (
                                4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) + 
                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) +
                                (timeStep*timeStep)
                                );
    Dufour_A_k_hat(5, 5) =    (4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) - 
                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) -                                 
                                (timeStep*timeStep))/                                                                                                       
                                (
                                4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) + 
                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) +
                                (timeStep*timeStep)
                                );                    
                                                                                
    ///Fill Dufour_B_k_hat
    //State Equation one, phases A,B,C: top submatrix
    Dufour_B_k_hat(0, 0) =  (timeStep*timeStep)/   
                            (4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) +
                            2. * timeStep * (**mCapacitance)(0, 0) * (**mResistance)(0, 0) +
                            (timeStep * timeStep));
    Dufour_B_k_hat(1, 1) =  (timeStep*timeStep)/   
                            (4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) +
                            2. * timeStep * (**mCapacitance)(1, 1) * (**mResistance)(1, 1) +
                            (timeStep * timeStep));
    Dufour_B_k_hat(2, 2) =  (timeStep*timeStep)/   
                            (4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) +
                            2. * timeStep * (**mCapacitance)(2, 2) * (**mResistance)(2, 2) +
                            (timeStep * timeStep));

    //State Equation two, phases A,B,C: bottom submatrix
    Dufour_B_k_hat(3, 0) =  (timeStep * 2. * (**mCapacitance)(0, 0))/   
                            (4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) +
                            2. * timeStep * (**mCapacitance)(0, 0) * (**mResistance)(0, 0) +
                            (timeStep * timeStep));
    Dufour_B_k_hat(4, 1) =  (timeStep * 2. * (**mCapacitance)(1, 1))/   
                            (4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) +
                            2. * timeStep * (**mCapacitance)(1, 1) * (**mResistance)(1, 1) +
                            (timeStep * timeStep));
    Dufour_B_k_hat(5, 2) =  (timeStep * 2. * (**mCapacitance)(2, 2))/   
                            (4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) +
                            2. * timeStep * (**mCapacitance)(2, 2) * (**mResistance)(2, 2) +
                            (timeStep * timeStep));

    Dufour_B_k_n_hat = Dufour_B_k_hat;

    Dufour_C_k_n(0, 3) = 1.;
    Dufour_C_k_n(1, 4) = 1.;
    Dufour_C_k_n(2, 5) = 1.;

	Dufour_W_k_n = Dufour_C_k_n * Dufour_B_k_hat;

	///FIXME:	mIntfCurrent is state 2 and is potentially directly initialized by other initialization methodes (e.g. FromNodesAndTerminals).
	///			State 1, which is Voltage over the capacitor, is not directly initialized and has to be calculated from the states. This is why
	///			the current must be reset as it would be altered as well. However, the old value of state one (time step "-1") is unknown or "zero"
	///			in this case, so calculation of state 1 would always assume zero as the past value of state 1 and also takes mIntfCurrent
	///			for the calculation ->State 1 is ahead of state 2 by one step, but also always (wrongly?) assumes past state 1 to be zero.
	///			How to handle properly?
	State(3, 0) = (**mIntfCurrent)(0, 0);
	State(4, 0) = (**mIntfCurrent)(1, 0);
	State(5, 0) = (**mIntfCurrent)(2, 0);
	ssnUpdateState();
	State(3, 0) = (**mIntfCurrent)(0, 0);
	State(4, 0) = (**mIntfCurrent)(1, 0);
	State(5, 0) = (**mIntfCurrent)(2, 0);

	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	mSLog->info(
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::matrixToString(**mIntfVoltage),
		Logger::matrixToString(**mIntfCurrent));
	mSLog->flush();
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), Dufour_W_k_n(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), Dufour_W_k_n(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), Dufour_W_k_n(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), Dufour_W_k_n(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), Dufour_W_k_n(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), Dufour_W_k_n(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), Dufour_W_k_n(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), Dufour_W_k_n(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), Dufour_W_k_n(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), Dufour_W_k_n(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 1), Dufour_W_k_n(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 2), Dufour_W_k_n(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 0), Dufour_W_k_n(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), Dufour_W_k_n(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 2), Dufour_W_k_n(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 0), Dufour_W_k_n(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 1), Dufour_W_k_n(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), Dufour_W_k_n(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -Dufour_W_k_n(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 1), -Dufour_W_k_n(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 2), -Dufour_W_k_n(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 0), -Dufour_W_k_n(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), -Dufour_W_k_n(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 2), -Dufour_W_k_n(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 0), -Dufour_W_k_n(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 1), -Dufour_W_k_n(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), -Dufour_W_k_n(2, 2));


		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -Dufour_W_k_n(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 1), -Dufour_W_k_n(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 2), -Dufour_W_k_n(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 0), -Dufour_W_k_n(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), -Dufour_W_k_n(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 2), -Dufour_W_k_n(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 0), -Dufour_W_k_n(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 1), -Dufour_W_k_n(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), -Dufour_W_k_n(2, 2));
	}
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	// Update internal state
	yHistory = Dufour_C_k_n * (Dufour_A_k_hat * State +  Dufour_B_k_hat * **mIntfVoltage);
	
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), yHistory(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 1), yHistory(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 2), yHistory(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), -yHistory(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 1), -yHistory(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 2), -yHistory(2, 0));
	}
	mSLog->debug(
		"\nHistory current term (mnaApplyRightSideVectorStamp): {:s}",
		Logger::matrixToString(yHistory));
	mSLog->flush();
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on L, but then we'd have to modify the system matrix anyway
	prevStepDependencies.push_back(attribute("v_intf"));
	prevStepDependencies.push_back(attribute("i_intf"));
	modifiedAttributes.push_back(attribute("right_vector"));
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaPreStep(Real time, Int timeStepCount) {
	mnaApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph3::SSN::Full_Serial_RLC::ssnUpdateState()
{
	State = Dufour_A_k_hat * State + Dufour_B_k_hat * Dufour_u_n_t + Dufour_B_k_n_hat * **mIntfVoltage;
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaUpdateVoltage(**leftVector);
	mnaUpdateCurrent(**leftVector);
    ssnUpdateState();
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	Dufour_u_n_t = **mIntfVoltage;
	**mIntfVoltage = Matrix::Zero(3, 1);
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
	mSLog->debug(
		"\nUpdate Voltage: {:s}",
		Logger::matrixToString(**mIntfVoltage)
	);
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaUpdateCurrent(const Matrix& leftVector) {
    **mIntfCurrent = yHistory + Dufour_W_k_n * **mIntfVoltage;
	mSLog->debug(
		"\nUpdate Current: {:s}",
		Logger::matrixToString(**mIntfCurrent)
	);
	mSLog->flush();
}

