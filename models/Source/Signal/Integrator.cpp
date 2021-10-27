/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/Integrator.h>

using namespace CPS;
using namespace CPS::Signal;

Integrator::Integrator(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel) {

    addAttribute<Real>("input_ref", Flags::read | Flags::write);

    addAttribute<Real>("input_prev", &mInputPrev, Flags::read | Flags::write);
    addAttribute<Real>("state_prev", &mStatePrev, Flags::read | Flags::write);
    addAttribute<Real>("output_prev", &mOutputPrev, Flags::read | Flags::write);

    addAttribute<Real>("input_curr", &mInputCurr, Flags::read | Flags::write);
    addAttribute<Real>("state_curr", &mStateCurr, Flags::read | Flags::write);
    addAttribute<Real>("output_curr", &mOutputCurr, Flags::read | Flags::write);
}

void Integrator::setParameters(Real timestep) {
    mTimeStep = timestep;

    mSLog->info("Integration step = {}", mTimeStep);
}

void Integrator::setInitialValues(Real input_init, Real state_init, Real output_init) {
	mInputCurr = input_init;
    mStateCurr = state_init;
    mOutputCurr = output_init;

    mSLog->info("Initial values:");
    mSLog->info("inputCurrInit = {}, stateCurrInit = {}, outputCurrInit = {}", mInputCurr, mStateCurr, mOutputCurr);
}

void Integrator::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(attribute("input_curr"));
	prevStepDependencies.push_back(attribute("output_curr"));
	modifiedAttributes.push_back(attribute("input_prev"));
    modifiedAttributes.push_back(attribute("output_prev"));
};

void Integrator::signalPreStep(Real time, Int timeStepCount) {
    mInputPrev = mInputCurr;
    mStatePrev = mStateCurr;
    mOutputPrev = mOutputCurr;
}

void Integrator::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(attribute("input_ref"));
	modifiedAttributes.push_back(attribute("input_curr"));
    modifiedAttributes.push_back(attribute("output_curr"));
};

void Integrator::signalStep(Real time, Int timeStepCount) {
    mInputCurr = attribute<Real>("input_ref")->get();

    mSLog->info("Time {}:", time);
    mSLog->info("Input values: inputCurr = {}, inputPrev = {}, statePrev = {}", mInputCurr, mInputPrev, mStatePrev);

    mStateCurr = mStatePrev + mTimeStep/2.0*mInputCurr + mTimeStep/2.0*mInputPrev;
    mOutputCurr = mStateCurr;

    mSLog->info("State values: stateCurr = {}", mStateCurr);
    mSLog->info("Output values: outputCurr = {}:", mOutputCurr);
}

Task::List Integrator::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
