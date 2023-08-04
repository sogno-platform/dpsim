/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/Droop.h>

using namespace CPS;
using namespace CPS::Signal;

Droop::Droop(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    mInputRef(Attribute<Real>::createDynamic("input_ref", mAttributes)),
    /// CHECK: Which of these really need to be attributes?

    //Previous
    mInputPrev(Attribute<Matrix>::create("input_prev", mAttributes, Matrix::Zero(3,1))),
    mStatePrev(Attribute<Matrix>::create("state_prev", mAttributes, Matrix::Zero(1,1))),
    mOutputPrev(Attribute<Matrix>::create("output_prev", mAttributes, Matrix::Zero(1,1))),

    //Currently
    mInputCurr(Attribute<Matrix>::create("input_curr", mAttributes, Matrix::Zero(3,1))),
    mStateCurr(Attribute<Matrix>::create("state_curr", mAttributes, Matrix::Zero(1,1))),
    mOutputCurr(Attribute<Matrix>::create("output_curr", mAttributes, Matrix::Zero(1,1))), 
    
    //Measured from system
    mVc_d(Attribute<Real>::createDynamic("Vc_d", mAttributes)),
	mVc_q(Attribute<Real>::createDynamic("Vc_q", mAttributes)),
	mIrc_d(Attribute<Real>::createDynamic("Irc_d", mAttributes)),
	mIrc_q(Attribute<Real>::createDynamic("Irc_q", mAttributes)),
    mOmega(Attribute<Real>::createDynamic("OmegaSystem", mAttributes)),
    mPowerInst(Attribute<Real>::createDynamic("PowerSystem", mAttributes))
    { 
	    mSLog->info("Create {} {}", type(), name);
    }


// set Parameters
void Droop::setParameters(Real powerSet, Real omegaNom) {
   
    mOmegaNom = omegaNom;
    mPowerSet = powerSet;
  
    mSLog->info("Omega Nominal = {}, Power Set Point = {}", mOmegaNom, mPowerSet);
}

//setter for controller parameters and setting up of system matrices
void Droop::setControllerParameters(Real taup, Real taui, Real mp) {
    
    mTaup = taup;
    mTaui = taui;
    mMp = mp;
	
    mSLog->info("Taup = {}, Taui = {}, Mp = {}", mMp, mTaui, mTaup);

    /// [x] = omegaSystem
    /// [y] = omegaSystem
    /// [u] = omegaNominal, powerSystem, powerSet

    mA <<  -1/mTaup;
    mB <<  1/mTaup, -mMp/mTaup, mMp/mTaup;
    mC <<  1;
    mD <<  0, 0, 0;

    mSLog->info("State space matrices:");
    mSLog->info("A = \n{}", mA);
    mSLog->info("B = \n{}", mB);
    mSLog->info("C = \n{}", mC);
    mSLog->info("D = \n{}", mD);
}

void Droop::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    mSLog->info("Integration step = {}", mTimeStep);
}

void Droop::setInitialStateValues(Real omegaInit, Real powerInit) {
	
    mOmegaInit = omegaInit;
	mPowerInit = powerInit;

	mSLog->info("Initial State Value Parameters:");
	mSLog->info("Omega Init = {}, Power Init = {}", omegaInit, powerInit);
}

//Creating state space model out of the variables
void Droop::initializeStateSpaceModel(Real omegaNom, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	mTimeStep = timeStep;
	mOmegaNom = omegaNom;

	// initialization of input --> [u]
	**mInputCurr << mOmegaNom, **mOmega, **mPowerInst;
	mSLog->info("Initialization of input: \n" + Logger::matrixToString(**mInputCurr));

	// initialization of states --> [x]
	**mStateCurr << mOmegaInit;
	mSLog->info("Initialization of states: \n" + Logger::matrixToString(**mStateCurr));

	// initialization of output --> [y]
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	mSLog->info("Initialization of output: \n" + Logger::matrixToString(**mOutputCurr));
}

void Droop::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void Droop::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void Droop::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	
    **mPowerInst = **mVc_d * **mIrc_d + **mVc_d * **mIrc_d; 
    attributeDependencies.push_back(mOmega);
	attributeDependencies.push_back(mPowerInst);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void Droop::signalStep(Real time, Int timeStepCount) {
    // get current inputs
	**mInputCurr << mOmegaNom, **mOmega, **mPowerInst;
    mSLog->debug("Time {}\n: inputCurr = \n{}\n , inputPrev = \n{}\n , statePrev = \n{}", time, **mInputCurr, **mInputPrev, **mStatePrev);

	// calculate new states
	**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
	mSLog->debug("stateCurr = \n {}", **mStateCurr);

	// calculate new outputs
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	mSLog->debug("Output values: outputCurr = \n{}", **mOutputCurr);
}

Task::List Droop::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
