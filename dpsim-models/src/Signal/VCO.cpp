/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/VCO.h>

using namespace CPS;
using namespace CPS::Signal;

VCO::VCO(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    mInputRef(mAttributes->createDynamic<Real>("input_ref")),
    mOutputRef(mAttributes->createDynamic<Real>("output_ref")),
    /// CHECK: Which of these really need to be attributes?
    /*
    mInputPrev(mAttributes->create<Real>("input_prev", 0)),
    mStatePrev(mAttributes->create<Real>("state_prev", 0)),
    mOutputPrev(mAttributes->create<Real>("output_prev", 0)),
    mInputCurr(mAttributes->create<Real>("input_curr", 0)),
    mStateCurr(mAttributes->create<Real>("state_curr", 0)),
    mOutputCurr(mAttributes->create<Real>("output_curr", 0))
    */
    mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(1,1))),
    mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(1,1))),
    mOutputPrev(mAttributes->create<Matrix>("output_prev", Matrix::Zero(1,1))),

    mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(1,1))),
    mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(1,1))),
    mOutputCurr(mAttributes->create<Matrix>("output_curr", Matrix::Zero(1,1)))

    { }


void VCO::setParameters(Real omega) {
    // Input is OmegaNom
    // Output is Theta
    mOmega = omega;
    SPDLOG_LOGGER_INFO(mSLog, "Omega = {}", mOmega);

    (**mInputCurr)(0,0) = mOmega;
}

void VCO::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    SPDLOG_LOGGER_INFO(mSLog, "Integration step = {}", mTimeStep);
}

void VCO::setInitialValues(Real input_init, Real state_init, Real output_init) {
	(**mInputCurr)(0,0) = input_init;
    (**mStateCurr)(0,0) = state_init;
    (**mOutputCurr)(0,0) = output_init;

    SPDLOG_LOGGER_INFO(mSLog, "Initial values:");
    SPDLOG_LOGGER_INFO(mSLog, "inputCurrInit = ({}), stateCurrInit = ({}), outputCurrInit = ({})", (**mInputCurr)(0,0), (**mStateCurr)(0,0), (**mOutputCurr)(0,0));
}

void VCO::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void VCO::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void VCO::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mInputRef);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void VCO::signalStep(Real time, Int timeStepCount) {

    (**mInputCurr)(0,0) = **mInputRef; // frequency input from outside

	// [x] = phase
	// [u] = frequency input
	// [y] = phase (not used here)

    mA <<  0;
    mB <<  1;

    SPDLOG_LOGGER_DEBUG(mSLog, "Time {}\n: inputCurr = \n{}\n , inputPrev = \n{}\n , statePrev = \n{}", time, (**mInputCurr)(0,0), (**mInputPrev)(0,0), (**mStatePrev)(0,0));

    // calculate new states
    //**mStateCurr = (**mStatePrev + mTimeStep * **mInputCurr);
    **mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
    SPDLOG_LOGGER_DEBUG(mSLog, "stateCurr = \n {}", **mStateCurr);

    // calculate new outputs
    **mOutputCurr = **mStateCurr;
    **mOutputRef = (**mOutputCurr)(0,0);

    SPDLOG_LOGGER_DEBUG(mSLog, "Output values: outputCurr = \n{}", (**mOutputCurr)(0,0));
}

Task::List VCO::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
