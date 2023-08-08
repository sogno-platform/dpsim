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
    mInputRef(mAttributes->createDynamic<Real>("input_ref")),
    /// CHECK: Which of these really need to be attributes?

    //Previous
    mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(3,1))),
    mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(1,1))),
    mOutputPrev(mAttributes->create<Matrix>("output_prev", Matrix::Zero(1,1))),

    //Currently
    mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(3,1))),
    mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(1,1))),
    mOutputCurr(mAttributes->create<Matrix>("output_curr", Matrix::Zero(1,1))), 
    
    //Measured from system
    mVc_d(mAttributes->createDynamic<Real>("Vc_d")),
	mVc_q(mAttributes->createDynamic<Real>("Vc_q")),
	mIrc_d(mAttributes->createDynamic<Real>("Irc_d")),
	mIrc_q(mAttributes->createDynamic<Real>("Irc_q")),
    mOmega(mAttributes->createDynamic<Real>("OmegaSystem")),
    mPowerInst(mAttributes->createDynamic<Real>("PowerSystem"))
    
    { 
	    SPDLOG_LOGGER_INFO(mSLog,"Create {} {}", type(), name);
    }


// set Parameters
void Droop::setParameters(Real powerSet, Real omegaNom) {
   
    mOmegaNom = omegaNom;
    mPowerSet = powerSet;
  
    SPDLOG_LOGGER_INFO(mSLog, "Omega Nominal = {}, Power Set Point = {}", mOmegaNom, mPowerSet);
}

//setter for controller parameters and setting up of system matrices
void Droop::setControllerParameters(Real taup, Real taui, Real mp) {
    
    mTaup = taup;
    mTaui = taui;
    mMp = mp;
	
    SPDLOG_LOGGER_INFO(mSLog,"Taup = {}, Taui = {}, Mp = {}", mMp, mTaui, mTaup);

    /// [x] = omegaSystem
    /// [y] = omegaSystem
    /// [u] = omegaNominal, powerSystem, powerSet

    mA <<  -1/mTaup;
    mB <<  1/mTaup, -mMp/mTaup, mMp/mTaup;
    mC <<  1;
    mD <<  0, 0, 0;

    SPDLOG_LOGGER_INFO(mSLog,"State space matrices:");
    SPDLOG_LOGGER_INFO(mSLog,"A = \n{}", mA);
    SPDLOG_LOGGER_INFO(mSLog,"B = \n{}", mB);
    SPDLOG_LOGGER_INFO(mSLog,"C = \n{}", mC);
    SPDLOG_LOGGER_INFO(mSLog,"D = \n{}", mD);
}

void Droop::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    SPDLOG_LOGGER_INFO(mSLog,"Integration step = {}", mTimeStep);
}

void Droop::setInitialStateValues(Real omegaInit, Real powerInit) {
	
    mOmegaInit = omegaInit;
	mPowerInit = powerInit;

	SPDLOG_LOGGER_INFO(mSLog,"Initial State Value Parameters:");
	SPDLOG_LOGGER_INFO(mSLog,"Omega Init = {}, Power Init = {}", omegaInit, powerInit);
}

//Creating state space model out of the variables
void Droop::initializeStateSpaceModel(Real omegaNom, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	mTimeStep = timeStep;
	mOmegaNom = omegaNom;

	// initialization of input --> [u]
	**mInputCurr << mOmegaNom, **mOmega, **mPowerInst;
	SPDLOG_LOGGER_INFO(mSLog,"Initialization of input: \n" + Logger::matrixToString(**mInputCurr));

	// initialization of states --> [x]
	**mStateCurr << mOmegaInit;
	SPDLOG_LOGGER_INFO(mSLog,"Initialization of states: \n" + Logger::matrixToString(**mStateCurr));

	// initialization of output --> [y]
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	SPDLOG_LOGGER_INFO(mSLog,"Initialization of output: \n" + Logger::matrixToString(**mOutputCurr));
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
    SPDLOG_LOGGER_DEBUG(mSLog, "Time {}\n: inputCurr = \n{}\n , inputPrev = \n{}\n , statePrev = \n{}", time, **mInputCurr, **mInputPrev, **mStatePrev);

	// calculate new states
	**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
	SPDLOG_LOGGER_DEBUG(mSLog, "stateCurr = \n {}", **mStateCurr);

	// calculate new outputs
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	SPDLOG_LOGGER_DEBUG(mSLog, "Output values: outputCurr = \n{}", **mOutputCurr);
}

Task::List Droop::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
