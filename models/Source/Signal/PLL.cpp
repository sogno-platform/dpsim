/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/PLL.h>

using namespace CPS;
using namespace CPS::Signal;

PLL::PLL(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel) {

    addAttribute<Real>("input_ref", Flags::read | Flags::write);

    addAttribute<Matrix>("input_prev", &mInputPrev, Flags::read | Flags::write);
    addAttribute<Matrix>("state_prev", &mStatePrev, Flags::read | Flags::write);
    addAttribute<Matrix>("output_prev", &mOutputPrev, Flags::read | Flags::write);

    addAttribute<Matrix>("input_curr", &mInputCurr, Flags::read | Flags::write);
    addAttribute<Matrix>("state_curr", &mStateCurr, Flags::read | Flags::write);
    addAttribute<Matrix>("output_curr", &mOutputCurr, Flags::read | Flags::write);
}


void PLL::setParameters(Real kpPLL, Real kiPLL, Real omegaNom) {
    mKp = kpPLL;
    mKi = kiPLL;
    mOmegaNom = omegaNom;
    mSLog->info("Kp = {}, Ki = {}", mKp, mKi);

    // First entry of input vector is constant omega
    mInputCurr(0,0) = mOmegaNom;
}

void PLL::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    mSLog->info("Integration step = {}", mTimeStep);
}

void PLL::setInitialValues(Real input_init, Matrix state_init, Matrix output_init) {
	mInputCurr(1,0) = input_init;
    mStateCurr = state_init;
    mOutputCurr = output_init;

    mSLog->info("Initial values:");
    mSLog->info("inputCurrInit = ({}, {}), stateCurrInit = ({}, {}), outputCurrInit = ({}, {})", mInputCurr(0,0), mInputCurr(1,0), mInputPrev(0,0), mInputPrev(1,0), mStateCurr(0,0), mStateCurr(1,0), mStatePrev(0,0), mStatePrev(1,0));
}

void PLL::composeStateSpaceMatrices() {
    mA <<   0,  mKi,
            0,  0;
    mB <<   1,  mKp,
            0,  1;
    mC <<   1,  0,
            0,  1;
    mD <<   0,  0,
            0,  0;

    mSLog->info("State space matrices:");
    mSLog->info("A = \n{}", mA);
    mSLog->info("B = \n{}", mB);
    mSLog->info("C = \n{}", mC);
    mSLog->info("D = \n{}", mD);
}

void PLL::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(attribute("input_curr"));
	prevStepDependencies.push_back(attribute("output_curr"));
	modifiedAttributes.push_back(attribute("input_prev"));
    modifiedAttributes.push_back(attribute("output_prev"));
};

void PLL::signalPreStep(Real time, Int timeStepCount) {
    mInputPrev = mInputCurr;
    mStatePrev = mStateCurr;
    mOutputPrev = mOutputCurr;
}

void PLL::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(attribute("input_ref"));
	modifiedAttributes.push_back(attribute("input_curr"));
    modifiedAttributes.push_back(attribute("output_curr"));
};

void PLL::signalStep(Real time, Int timeStepCount) {
    mInputCurr(1,0) = attribute<Real>("input_ref")->get();

    mSLog->info("Time {}:", time);
    mSLog->info("Input values: inputCurr = ({}, {}), inputPrev = ({}, {}), stateCurr = ({}, {}), statePrev = ({}, {})", mInputCurr(0,0), mInputCurr(1,0), mInputPrev(0,0), mInputPrev(1,0), mStateCurr(0,0), mStateCurr(1,0), mStatePrev(0,0), mStatePrev(1,0));

    mStateCurr = Math::StateSpaceTrapezoidal(mStatePrev, mA, mB, mTimeStep, mInputCurr, mInputPrev);
    mOutputCurr = mC * mStateCurr + mD * mInputCurr;

    mSLog->info("State values: stateCurr = ({}, {})", mStateCurr(0,0), mStateCurr(1,0));
    mSLog->info("Output values: outputCurr = ({}, {}):", mOutputCurr(0,0), mOutputCurr(1,0));
}

Task::List PLL::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
