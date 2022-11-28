/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/EMT/EMT_Ph3_varResSwitch.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::varResSwitch::varResSwitch(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Real>(uid, name, logLevel) {
	setTerminalNumber(2);
	**mIntfVoltage = Matrix::Zero(1,1);
	**mIntfCurrent = Matrix::Zero(1,1);

	mOpenResistance = Attribute<Matrix>::create("R_open", mAttributes);
	mClosedResistance = Attribute<Matrix>::create("R_closed", mAttributes);
	mSwitchClosed = Attribute<Bool>::create("is_closed", mAttributes);
}

SimPowerComp<Real>::Ptr EMT::Ph3::varResSwitch::clone(String name) {
	auto copy = varResSwitch::make(name, mLogLevel);
	copy->setParameters(**mOpenResistance, **mClosedResistance, **mSwitchClosed);
	return copy;
}

void EMT::Ph3::varResSwitch::initializeFromNodesAndTerminals(Real frequency) {

	Matrix impedance = (**mSwitchClosed) ? **mClosedResistance : **mOpenResistance;
	MatrixComp vInitABC = MatrixComp::Zero(3, 1);
	vInitABC(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	vInitABC(1, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_B;
	vInitABC(2, 0) = vInitABC(0, 0) * SHIFT_TO_PHASE_C;
	**mIntfVoltage = vInitABC.real();
	**mIntfCurrent = (impedance.inverse() * vInitABC).real();

	mSLog->info(
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

void EMT::Ph3::varResSwitch::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	// mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::varResSwitch::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Matrix conductance = (**mSwitchClosed) ?
		(**mClosedResistance).inverse() : (**mOpenResistance).inverse();

	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), conductance(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), conductance(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), -conductance(2, 2));


		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), -conductance(2, 2));
	}
	mSLog->info(
		"\nConductance matrix: {:s}",
		Logger::matrixToString(conductance));
}

void EMT::Ph3::varResSwitch::mnaApplySwitchSystemMatrixStamp(Bool closed, Matrix& systemMatrix, Int freqIdx) {
	Matrix conductance = (closed) ?
		(**mClosedResistance).inverse() : (**mOpenResistance).inverse();

	// Set diagonal entries
	if (terminalNotGrounded(0)) {
		// set upper left block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(0, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(0, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(0, 2), conductance(2, 2));
	}
	if (terminalNotGrounded(1)) {
		// set buttom right block, 3x3 entries
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 0), conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 1), conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(1, 2), conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 0), conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 1), conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(1, 2), conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 0), conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 1), conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(1, 2), conductance(2, 2));
	}
	// Set off diagonal blocks, 2x3x3 entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 0), matrixNodeIndex(1, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 1), matrixNodeIndex(1, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0, 2), matrixNodeIndex(1, 2), -conductance(2, 2));


		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 0), -conductance(0, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 1), -conductance(0, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 0), matrixNodeIndex(0, 2), -conductance(0, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 0), -conductance(1, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 1), -conductance(1, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 1), matrixNodeIndex(0, 2), -conductance(1, 2));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 0), -conductance(2, 0));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 1), -conductance(2, 1));
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1, 2), matrixNodeIndex(0, 2), -conductance(2, 2));
	}

	mSLog->info(
		"\nConductance matrix: {:s}",
		Logger::matrixToString(conductance));
}

void EMT::Ph3::varResSwitch::mnaApplyRightSideVectorStamp(Matrix& rightVector) { }

void EMT::Ph3::varResSwitch::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
	Attribute<Matrix>::Ptr &leftVector) {

	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph3::varResSwitch::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector)  {
	mnaUpdateVoltage(*leftVector);
	mnaUpdateCurrent(*leftVector);
}

void EMT::Ph3::varResSwitch::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
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
}

void EMT::Ph3::varResSwitch::mnaUpdateCurrent(const Matrix& leftVector) {
	**mIntfCurrent = (**mSwitchClosed) ?
		(**mClosedResistance).inverse() * **mIntfVoltage:
		(**mOpenResistance).inverse() * **mIntfVoltage;
}

Bool EMT::Ph3::varResSwitch::hasParameterChanged() {
//Get present state
Bool presentState=this->mnaIsClosed();

// Check if state of switch changed from open to closed
if (!(mPrevState == presentState)) {
	// Switch is closed : change with 1/mDeltaRes
	if (this->mnaIsClosed()==true) {
		// **mClosedResistance= 1./mDeltaRes*mPrevRes;
		**mClosedResistance= mDeltaResClosed*mPrevRes;	
		mPrevRes= **mClosedResistance;
		// check if target value is reached
		if ((**mClosedResistance)(0,0) < mInitClosedRes(0,0)) {
			**mClosedResistance= mInitClosedRes;
			mPrevRes= **mClosedResistance;
			mPrevState= this->mnaIsClosed();
		}
	}
	// Switch is opened : change with mDeltaRes
	else if (this->mnaIsClosed()==false) {
		**mOpenResistance= mDeltaResOpen*mPrevRes;		
		mPrevRes= **mOpenResistance;
		// check if target value is reached
		if ( (**mOpenResistance)(0,0) > mInitOpenRes(0,0)) {
			**mOpenResistance= mInitOpenRes;
			mPrevRes= **mOpenResistance;
			mPrevState= this->mnaIsClosed();
		}
	}		
	return 1; //recompute system matrix
}
else{
	return 0; // do not recompute system matrix
	}
}

void EMT::Ph3::varResSwitch::setInitParameters(Real timestep) {
	//Define variables for the transition
	mDeltaResClosed= 0;
	// mDeltaResOpen = 1.5; // assumption for 1ms step size
	mDeltaResOpen= 0.5*timestep/0.001 + 1;
	mPrevState= **mSwitchClosed;
	mPrevRes= (**mSwitchClosed) ? **mClosedResistance : **mOpenResistance;
	mInitClosedRes=**mClosedResistance;
	mInitOpenRes=**mOpenResistance;
}