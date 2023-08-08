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
    mOutputRef(mAttributes->createDynamic<Real>("output_ref")),

    mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(3,1))),
    mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(1,1))),
    mOutputPrev(mAttributes->create<Matrix>("output_prev", Matrix::Zero(1,1))),
    mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(3,1))),
    mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(1,1))),
    mOutputCurr(mAttributes->create<Matrix>("output_curr", Matrix::Zero(1,1))) {}


// set Parameters
void Droop::setParameters(Real powerSet, Real omegaNom) {

    mOmegaNom = omegaNom;
    mPowerSet = powerSet;
  
    SPDLOG_LOGGER_INFO(mSLog, "Grid Nominal Omega = {}, Power Set Point = {}", mOmegaNom, mPowerSet);
}

//setter for controller parameters and setting up of system matrices
void Droop::setControllerParameters(Real m_p, Real tau_p, Real tau_l) {
    
    mM_p = m_p;    
    mTau_p = tau_p;
    mTau_l = tau_l;

    SPDLOG_LOGGER_INFO(mSLog,"m_p = {}, tau_p = {}, tau_l = {}, ", mM_p, mTau_p, mTau_l);

    /// [x] = omega
    /// [u] = PowerInst, PowerSet, omegaNom
    /// [y] = omega

    mA <<  -1/mTau_p;
    mB <<  1/mTau_p, -mM_p/mTau_p, mM_p/mTau_p;
    mC <<  1;
    mD <<  0, 0, 0;

    SPDLOG_LOGGER_INFO(mSLog,"State space matrices:");
    SPDLOG_LOGGER_INFO(mSLog,"A = \n{}", mA);
    SPDLOG_LOGGER_INFO(mSLog,"B = \n{}", mB);
    SPDLOG_LOGGER_INFO(mSLog,"C = \n{}", mC);
    SPDLOG_LOGGER_INFO(mSLog,"D = \n{}", mD);
}

// setter for simulation time
void Droop::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    SPDLOG_LOGGER_INFO(mSLog,"Integration step = {}", mTimeStep);
}

// initialize state values of matrices
void Droop::setInitialStateValues(Matrix input_init, Matrix state_init, Matrix output_init) {	
	**mInputCurr = input_init; /// [u] = PowerInst, PowerSet, omegaNom
    **mStateCurr = state_init;  /// [x] = omega
    **mOutputCurr = output_init; /// [y] = omega

	SPDLOG_LOGGER_INFO(mSLog,"Initial State Value Parameters:");
	SPDLOG_LOGGER_INFO(mSLog,"Omega Init = {}, Power Init = {}", **mStateCurr(0,0), **mInputCurr(0,0));
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
	attributeDependencies.push_back(mInputRef);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void Droop::signalStep(Real time, Int timeStepCount) {
    
    (**mInputCurr)(0,0) = **mInputRef;

    SPDLOG_LOGGER_INFO(mSLog, "Time {}:", time);
    SPDLOG_LOGGER_INFO(mSLog, "Input values: inputCurr = ({}, {}), inputPrev = ({}, {}), stateCurr = ({}, {}), statePrev = ({}, {})", (**mInputCurr)(0,0), (**mInputCurr)(1,0), (**mInputPrev)(0,0), (**mInputPrev)(1,0), (**mStateCurr)(0,0), (**mStateCurr)(1,0), (**mStatePrev)(0,0), (**mStatePrev)(1,0));

    **mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
    **mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;

    **mOutputRef=(**mOutputCurr)(0,0);

    SPDLOG_LOGGER_INFO(mSLog, "State values: stateCurr = ({}, {})", (**mStateCurr)(0,0), (**mStateCurr)(1,0));
    SPDLOG_LOGGER_INFO(mSLog, "Output values: outputCurr = ({}, {}):", (**mOutputCurr)(0,0), (**mOutputCurr)(1,0));
}

Task::List Droop::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
