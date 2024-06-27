/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_SSN_Full_Serial_RLC.h>

using namespace CPS;

EMT::Ph3::SSN::Full_Serial_RLC::Full_Serial_RLC(String uid, String name, Logger::Level logLevel)
	: 	MNASimPowerComp<Real>(uid, name, true, true, logLevel),
		Base::Ph3::Resistor(mAttributes),
		Base::Ph3::Inductor(mAttributes),
		Base::Ph3::Capacitor(mAttributes) {

	mPhaseType = PhaseType::ABC;

	**mIntfVoltage = Matrix::Zero(3, 1);
	**mIntfCurrent = Matrix::Zero(3, 1);
	setTerminalNumber(2);
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


	SPDLOG_LOGGER_INFO(mSLog, "\nImpedance [Ohm]: {:s}",
		Logger::matrixCompToString(impedance));
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
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	updateMatrixNodeIndices();

	mState = Matrix::Zero(6, 1);
    mYHistory =  Matrix::Zero(3, 1);

    //Fill mDufourAKHat
    //State Equation one, phases A,B,C: top left submatrix
    mDufourAKHat(0, 0) = 1.-   ((2.*(timeStep*timeStep))/(4.* (**mInductance)(0, 0)* (**mCapacitance)(0, 0) +
                                                            2. * timeStep * (**mCapacitance)(0, 0) * (**mResistance)(0, 0) +
                                                            timeStep*timeStep)
                                    );
    mDufourAKHat(1, 1) = 1.-   ((2.*(timeStep*timeStep))/(4.* (**mInductance)(1, 1)* (**mCapacitance)(1, 1) +
                                                            2. * timeStep * (**mCapacitance)(1, 1) * (**mResistance)(1, 1) +
                                                            timeStep*timeStep)
                                    );
    mDufourAKHat(2, 2) = 1.-   ((2.*(timeStep*timeStep))/(4.* (**mInductance)(2, 2)*(**mCapacitance)(2, 2) +
                                                            2. * timeStep * (**mCapacitance)(2, 2) * (**mResistance)(2, 2) +
                                                            timeStep*timeStep)
                                    );
    //State Equation one, phases A,B,C: top right submatrix
    mDufourAKHat(0, 3) = (timeStep/(2. * (**mCapacitance)(0, 0))) * (1. +     ((4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) - 
                                                                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) -
                                                                                (timeStep*timeStep))/
                                                                            (
                                                                                4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) + 
                                                                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) +
                                                                                (timeStep*timeStep))
                                                                                )
                                                                        );
    mDufourAKHat(1, 4) = (timeStep/(2. * (**mCapacitance)(1, 1))) * (1. +     ((4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) - 
                                                                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) -
                                                                                (timeStep*timeStep))/
                                                                            (
                                                                                4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) + 
                                                                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) +
                                                                                (timeStep*timeStep))
                                                                                )
                                                                        );
    mDufourAKHat(2, 5) = (timeStep/(2. * (**mCapacitance)(2, 2))) * (1. +     ((4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) - 
                                                                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) -
                                                                                (timeStep*timeStep))/
                                                                            (
                                                                                4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) + 
                                                                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) +
                                                                                (timeStep*timeStep))
                                                                                )
                                                                        );
    //State Equation two, phases A,B,C: bottom left submatrix
    mDufourAKHat(3, 0) = -  ((4. * (**mCapacitance)(0, 0) * timeStep)/(
                                4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) +
                                2. * timeStep* (**mCapacitance)(0, 0)* (**mResistance)(0, 0) +
                                (timeStep*timeStep)
                                ));
    mDufourAKHat(4, 1) = -  ((4. * (**mCapacitance)(1, 1) * timeStep)/(
                                4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) +
                                2. * timeStep* (**mCapacitance)(1, 1)* (**mResistance)(1, 1) +
                                (timeStep*timeStep)
                                ));
    mDufourAKHat(5, 2) = -  ((4. * (**mCapacitance)(2, 2) * timeStep)/(
                                4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) +
                                2. * timeStep* (**mCapacitance)(2, 2)* (**mResistance)(2, 2) +
                                (timeStep*timeStep)
                                ));
    //State Equation two, phases A,B,C: bottom right submatrix
    mDufourAKHat(3, 3) =    (4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) - 
                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) -                                 
                                (timeStep*timeStep))/                                                                                                       
                                (
                                4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) + 
                                2. * timeStep * (**mResistance)(0, 0) * (**mCapacitance)(0, 0) +
                                (timeStep*timeStep)
                                );
    mDufourAKHat(4, 4) =    (4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) - 
                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) -                                 
                                (timeStep*timeStep))/                                                                                                       
                                (
                                4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) + 
                                2. * timeStep * (**mResistance)(1, 1) * (**mCapacitance)(1, 1) +
                                (timeStep*timeStep)
                                );
    mDufourAKHat(5, 5) =    (4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) - 
                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) -                                 
                                (timeStep*timeStep))/                                                                                                       
                                (
                                4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) + 
                                2. * timeStep * (**mResistance)(2, 2) * (**mCapacitance)(2, 2) +
                                (timeStep*timeStep)
                                );                    
                                                                                
    ///Fill mDufourBKHat
    //State Equation one, phases A,B,C: top submatrix
    mDufourBKHat(0, 0) =  (timeStep*timeStep)/   
                            (4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) +
                            2. * timeStep * (**mCapacitance)(0, 0) * (**mResistance)(0, 0) +
                            (timeStep * timeStep));
    mDufourBKHat(1, 1) =  (timeStep*timeStep)/   
                            (4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) +
                            2. * timeStep * (**mCapacitance)(1, 1) * (**mResistance)(1, 1) +
                            (timeStep * timeStep));
    mDufourBKHat(2, 2) =  (timeStep*timeStep)/   
                            (4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) +
                            2. * timeStep * (**mCapacitance)(2, 2) * (**mResistance)(2, 2) +
                            (timeStep * timeStep));

    //State Equation two, phases A,B,C: bottom submatrix
    mDufourBKHat(3, 0) =  (timeStep * 2. * (**mCapacitance)(0, 0))/   
                            (4. * (**mInductance)(0, 0) * (**mCapacitance)(0, 0) +
                            2. * timeStep * (**mCapacitance)(0, 0) * (**mResistance)(0, 0) +
                            (timeStep * timeStep));
    mDufourBKHat(4, 1) =  (timeStep * 2. * (**mCapacitance)(1, 1))/   
                            (4. * (**mInductance)(1, 1) * (**mCapacitance)(1, 1) +
                            2. * timeStep * (**mCapacitance)(1, 1) * (**mResistance)(1, 1) +
                            (timeStep * timeStep));
    mDufourBKHat(5, 2) =  (timeStep * 2. * (**mCapacitance)(2, 2))/   
                            (4. * (**mInductance)(2, 2) * (**mCapacitance)(2, 2) +
                            2. * timeStep * (**mCapacitance)(2, 2) * (**mResistance)(2, 2) +
                            (timeStep * timeStep));

    mDufourBKNHat = mDufourBKHat;

    mDufourCKN(0, 3) = 1.;
    mDufourCKN(1, 4) = 1.;
    mDufourCKN(2, 5) = 1.;

	mDufourWKN = mDufourCKN * mDufourBKHat;

	///FIXME:	mIntfCurrent is state 2 and is potentially directly initialized by other initialization methodes (e.g. FromNodesAndTerminals).
	///			State 1, which is Voltage over the capacitor, is not directly initialized and has to be calculated from the states. This is why
	///			the current must be reset as it would be altered as well. However, the old value of state one (time step "-1") is unknown or "zero"
	///			in this case, so calculation of state 1 would always assume zero as the past value of state 1 and also takes mIntfCurrent
	///			for the calculation ->State 1 is ahead of state 2 by one step, but also always (wrongly?) assumes past state 1 to be zero.
	///			How to handle properly?
	mState(3, 0) = (**mIntfCurrent)(0, 0);
	mState(4, 0) = (**mIntfCurrent)(1, 0);
	mState(5, 0) = (**mIntfCurrent)(2, 0);
	ssnUpdateState();
	mState(3, 0) = (**mIntfCurrent)(0, 0);
	mState(4, 0) = (**mIntfCurrent)(1, 0);
	mState(5, 0) = (**mIntfCurrent)(2, 0);

	**mRightVector = Matrix::Zero(leftVector->get().rows(), 1);

	SPDLOG_LOGGER_INFO(mSLog,
		"\n--- MNA initialization ---"
		"\nInitial voltage {:s}"
		"\nInitial current {:s}"
		"\n--- MNA initialization finished ---",
		Logger::matrixToString(**mIntfVoltage),
		Logger::matrixToString(**mIntfCurrent));
	mSLog->flush();
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompApplySystemMatrixStamp(SparseMatrixRow& systemMatrix) {
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), mDufourWKN(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), mDufourWKN(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), mDufourWKN(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), mDufourWKN(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), mDufourWKN(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), mDufourWKN(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), mDufourWKN(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), mDufourWKN(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), mDufourWKN(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), mDufourWKN(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 1), mDufourWKN(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 2), mDufourWKN(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 0), mDufourWKN(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), mDufourWKN(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 2), mDufourWKN(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 0), mDufourWKN(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 1), mDufourWKN(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), mDufourWKN(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -mDufourWKN(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 1), -mDufourWKN(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 2), -mDufourWKN(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 0), -mDufourWKN(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), -mDufourWKN(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 2), -mDufourWKN(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 0), -mDufourWKN(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 1), -mDufourWKN(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), -mDufourWKN(2, 2));


		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -mDufourWKN(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 1), -mDufourWKN(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 2), -mDufourWKN(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 0), -mDufourWKN(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), -mDufourWKN(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 2), -mDufourWKN(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 0), -mDufourWKN(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 1), -mDufourWKN(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), -mDufourWKN(2, 2));
	}
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompApplyRightSideVectorStamp(Matrix& rightVector) {
	// Update history term
	mYHistory = mDufourCKN * (mDufourAKHat * mState +  mDufourBKHat * **mIntfVoltage);
	
	if (terminalNotGrounded(0)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 0), mYHistory(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 1), mYHistory(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(0, 2), mYHistory(2, 0));
	}
	if (terminalNotGrounded(1)) {
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 0), -mYHistory(0, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 1), -mYHistory(1, 0));
		Math::setVectorElement(rightVector, matrixNodeIndex(1, 2), -mYHistory(2, 0));
	}
	SPDLOG_LOGGER_DEBUG(mSLog,
		"\nHistory current term (mnaCompApplyRightSideVectorStamp): {:s}",
		Logger::matrixToString(mYHistory));
	mSLog->flush();
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	// actually depends on L,C, but then we'd have to modify the system matrix anyway
	prevStepDependencies.push_back(mIntfVoltage);
	prevStepDependencies.push_back(mIntfCurrent);
	modifiedAttributes.push_back(mRightVector);
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompPreStep(Real time, Int timeStepCount) {
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) {
	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(mIntfVoltage);
	modifiedAttributes.push_back(mIntfCurrent);
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaCompUpdateVoltage(**leftVector);
	mnaCompUpdateCurrent(**leftVector);
    ssnUpdateState();
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompUpdateVoltage(const Matrix& leftVector) {
	// v1 - v0
	mDufourUNT = **mIntfVoltage;
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
	SPDLOG_LOGGER_DEBUG(mSLog,
		"\nUpdate Voltage: {:s}",
		Logger::matrixToString(**mIntfVoltage));
	mSLog->flush();
}

void EMT::Ph3::SSN::Full_Serial_RLC::mnaCompUpdateCurrent(const Matrix& leftVector) {
    **mIntfCurrent = mYHistory + mDufourWKN * **mIntfVoltage;

	SPDLOG_LOGGER_DEBUG(mSLog,
		"\nUpdate Current: {:s}",
		Logger::matrixToString(**mIntfCurrent)
	);
	mSLog->flush();
}

void EMT::Ph3::SSN::Full_Serial_RLC::setParameters(Matrix resistance, Matrix inductance, Matrix capacitance)
{
    **mInductance = inductance;
    **mCapacitance = capacitance;
    **mResistance = resistance;
    mParametersSet = true;
}

void EMT::Ph3::SSN::Full_Serial_RLC::ssnUpdateState()
{
	mState = mDufourAKHat * mState + mDufourBKHat * mDufourUNT + mDufourBKNHat * **mIntfVoltage;
}