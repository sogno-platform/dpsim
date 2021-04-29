/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/DP/DP_Ph1_varResSwitch.h>

using namespace CPS;

DP::Ph1::varResSwitch::varResSwitch(String uid, String name, Logger::Level logLevel)
	: SimPowerComp<Complex>(uid, name, logLevel) {
	setTerminalNumber(2);
    mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Real>("R_open", &mOpenResistance, Flags::read | Flags::write);
	addAttribute<Real>("R_closed", &mClosedResistance, Flags::read | Flags::write);
	addAttribute<Bool>("is_closed", &mIsClosed, Flags::read | Flags::write);

	// addAttribute<Real>("Res", &mResistance, Flags::read | Flags::write);
}

SimPowerComp<Complex>::Ptr DP::Ph1::varResSwitch::clone(String name) {
	auto copy = varResSwitch::make(name, mLogLevel);
	copy->setParameters(mOpenResistance, mClosedResistance, mIsClosed);
	return copy;
}

void DP::Ph1::varResSwitch::initializeFromNodesAndTerminals(Real frequency) {

	//Switch Resistance
	Real impedance = (mIsClosed) ? mClosedResistance : mOpenResistance;

	mIntfVoltage(0,0) = initialSingleVoltage(1) - initialSingleVoltage(0);
	mIntfCurrent(0,0)  = mIntfVoltage(0,0) / impedance;

	mPrevState= this->mnaIsClosed();

	// Resistance step per timestep
	// i.e for a complete change over 10 timesteps
	mDeltaRes= (mOpenResistance - mClosedResistance) / 10;
}

// #### MNA functions ####

void DP::Ph1::varResSwitch::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateMatrixNodeIndices();

	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::varResSwitch::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	Complex conductance = (mIsClosed) ?
		Complex( 1./mClosedResistance, 0 ) : Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), conductance);
	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -conductance);
	}
}

void DP::Ph1::varResSwitch::mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed) {
	Complex conductance = (closed) ?
		Complex( 1./mClosedResistance, 0 ) :
		Complex( 1./mOpenResistance, 0 );

	// Set diagonal entries
	if (terminalNotGrounded(0))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), conductance);
	if (terminalNotGrounded(1))
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), conductance);

	// Set off diagonal entries
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), -conductance);
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), -conductance);
	}

	mSLog->info("-- Stamp ---");
	if (terminalNotGrounded(0))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), matrixNodeIndex(0), matrixNodeIndex(0));
	if (terminalNotGrounded(1))
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(conductance), matrixNodeIndex(1), matrixNodeIndex(1));
	if (terminalNotGrounded(0) && terminalNotGrounded(1)) {
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), matrixNodeIndex(0), matrixNodeIndex(1));
		mSLog->info("Add {:s} to system at ({:d},{:d})", Logger::complexToString(-conductance), matrixNodeIndex(1), matrixNodeIndex(0));
	}
}

void DP::Ph1::varResSwitch::mnaApplyRightSideVectorStamp(Matrix& rightVector) {}

// void DP::Ph1::varResSwitch::mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
// 	// add pre-step dependencies of component itself
// 	// prevStepDependencies.push_back(attribute("i_intf"));
// 	// prevStepDependencies.push_back(attribute("v_intf"));
// 	// modifiedAttributes.push_back(attribute("right_vector"));

// 	modifiedAttributes.push_back(attribute("R_open"));
// 	modifiedAttributes.push_back(attribute("R_closed"));
// }

// void DP::Ph1::varResSwitch::mnaPreStep(Real time, Int timeStepCount) {

//     // mnaApplyRightSideVectorStamp(mRightVector);

// 	//get present state
// 	Bool presentState=this->mnaIsClosed();

// 	// prevState wont be updated for the whole interval of the smooth transition
// 	// Check if state of switch changed and target value of resistance is not reached yet
// 	if(mPrevState != presentState) { 
// 		mTransitionCounter+=1;
// 		updateResistance();	
// 			if (mTransitionCounter > 10) {
// 				mPrevState= this->mnaIsClosed();
// 				mTransitionCounter=0;
// 	}
// 	}
// }

void DP::Ph1::varResSwitch::mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
	AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
	Attribute<Matrix>::Ptr &leftVector) {

	attributeDependencies.push_back(leftVector);
	modifiedAttributes.push_back(attribute("v_intf"));
	modifiedAttributes.push_back(attribute("i_intf"));
}

void DP::Ph1::varResSwitch::mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
	mnaUpdateVoltage(*leftVector);
	mnaUpdateCurrent(*leftVector);
}

void DP::Ph1::varResSwitch::mnaUpdateVoltage(const Matrix& leftVector) {
	// Voltage across component is defined as V1 - V0
	mIntfVoltage(0, 0) = 0;
	if (terminalNotGrounded(1)) mIntfVoltage(0,0) = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1));
	if (terminalNotGrounded(0)) mIntfVoltage(0,0) = mIntfVoltage(0,0) - Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::varResSwitch::mnaUpdateCurrent(const Matrix& leftVector) {
	mIntfCurrent(0,0) = (mIsClosed) ?
		mIntfVoltage(0,0) / mClosedResistance :
		mIntfVoltage(0,0) / mOpenResistance;
}

void DP::Ph1::varResSwitch::updateResistance() {

	mDeltaRes=5e3/10;

	// Check switch state to find out if resistance should be incremented or decremented
	if (this->mnaIsClosed()) {
		mClosedResistance= mOpenResistance - mDeltaRes * mTransitionResCounter;
	}
	else{
		mOpenResistance= mClosedResistance + mDeltaRes * mTransitionResCounter;
	}

}

Bool DP::Ph1::varResSwitch::hasParameterChanged() {
	
	//get present state
	Bool presentState=this->mnaIsClosed();

	mDeltaRes=5e3;

	Real factor = 0.9;

		if(mPrevState != presentState) { 
		
		if (this->mnaIsClosed()) {
				
			if (mTransitionResCounter == 0) {
			mClosedResistance= mOpenResistance - mDeltaRes * 0.97;
			mTransitionResCounter+=1;
			}
			else if (mTransitionResCounter == 1) {
			mClosedResistance= mOpenResistance - mDeltaRes * 0.03;
			mPrevState= this->mnaIsClosed();
			mTransitionResCounter=0;
			}

			return 1;
		}
		
		else{
			if (mTransitionResCounter == 0) {
			mOpenResistance= mClosedResistance + mDeltaRes * 0.97;
			mTransitionResCounter+=1;
			}
			else if (mTransitionResCounter == 1) {
			mOpenResistance= mClosedResistance + mDeltaRes * 0.03;
			mPrevState= this->mnaIsClosed();
			mTransitionResCounter=0;
			}

			return 1;

		}

	}

	else{
		return 0;
	}




	// // prevState wont be updated for the whole interval of the smooth transition
	// // Check if state of switch changed and target value of resistance is not reached yet
	// if(mPrevState != presentState) { 


	// 	if (this->mnaIsClosed()) {
			
	// 		mTransitionResCounter+=1;
	// 		mClosedResistance= mOpenResistance - mDeltaRes * mTransitionResCounter/2;

	// 		// updateResistance();
		
	// 		if (mTransitionResCounter == 2) {
	// 			mPrevState= this->mnaIsClosed();
	// 			mTransitionResCounter=0;
	// 		}

	// 		return 1;
	// 	}
		
	// 	else{ 
			
	// 		mTransitionResCounter+=1;
	// 		mOpenResistance= mClosedResistance + mDeltaRes * mTransitionResCounter /100;
			
					
	// 		if (mTransitionResCounter == 100) {
	// 			mPrevState= this->mnaIsClosed();
	// 			mTransitionResCounter=0;
	// 		}

	// 		return 1;	

	// 	}

	// }

	// else{
	// 	return 0;
	// }

}