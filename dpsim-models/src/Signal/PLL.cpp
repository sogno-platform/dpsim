/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/PLL.h>

using namespace CPS;
using namespace CPS::Signal;

PLL::PLL(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
    mInputRef(mAttributes->createDynamic<Real>("input_ref")),
    /// CHECK: Which of these really need to be attributes?
    mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(2,1))),
    mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(2,1))),
    mOutputPrev(mAttributes->create<Matrix>("output_prev", Matrix::Zero(2,1))),
    mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(2,1))),
    mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(2,1))),
    mOutputCurr(mAttributes->create<Matrix>("output_curr", Matrix::Zero(2,1))) { }


void PLL::setParameters(Real kpPLL, Real kiPLL, Real omegaNom) {
    mKp = kpPLL;
    mKi = kiPLL;
    mOmegaNom = omegaNom;
    SPDLOG_LOGGER_INFO(mSLog, "Kp = {}, Ki = {}", mKp, mKi);

    // First entry of input vector is constant omega
    (**mInputCurr)(0,0) = mOmegaNom;
}

void PLL::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    SPDLOG_LOGGER_INFO(mSLog, "Integration step = {}", mTimeStep);
}

void PLL::setInitialValues(Real input_init, Matrix state_init, Matrix output_init) {
	(**mInputCurr)(1,0) = input_init;
    **mStateCurr = state_init;
    **mOutputCurr = output_init;

    SPDLOG_LOGGER_INFO(mSLog, "Initial values:");
    SPDLOG_LOGGER_INFO(mSLog, "inputCurrInit = ({}, {}), stateCurrInit = ({}, {}), outputCurrInit = ({}, {})", (**mInputCurr)(0,0), (**mInputCurr)(1,0), (**mInputPrev)(0,0), (**mInputPrev)(1,0), (**mStateCurr)(0,0), (**mStateCurr)(1,0), (**mStatePrev)(0,0), (**mStatePrev)(1,0));
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

    SPDLOG_LOGGER_DEBUG(mSLog, "State space matrices:");
    SPDLOG_LOGGER_DEBUG(mSLog, "A = \n{}", mA);
    SPDLOG_LOGGER_DEBUG(mSLog, "B = \n{}", mB);
    SPDLOG_LOGGER_DEBUG(mSLog, "C = \n{}", mC);
    SPDLOG_LOGGER_DEBUG(mSLog, "D = \n{}", mD);
}

void PLL::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void PLL::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void PLL::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mInputRef);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void PLL::signalStep(Real time, Int timeStepCount) {
    (**mInputCurr)(1,0) = **mInputRef;

    SPDLOG_LOGGER_TRACE(mSLog, "Time {}:", time);
    SPDLOG_LOGGER_TRACE(mSLog, "Input values: inputCurr = ({}, {}), inputPrev = ({}, {}), stateCurr = ({}, {}), statePrev = ({}, {})", (**mInputCurr)(0,0), (**mInputCurr)(1,0), (**mInputPrev)(0,0), (**mInputPrev)(1,0), (**mStateCurr)(0,0), (**mStateCurr)(1,0), (**mStatePrev)(0,0), (**mStatePrev)(1,0));

    **mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
    **mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;

    SPDLOG_LOGGER_TRACE(mSLog, "State values: stateCurr = ({}, {})", (**mStateCurr)(0,0), (**mStateCurr)(1,0));
    SPDLOG_LOGGER_TRACE(mSLog, "Output values: outputCurr = ({}, {}):", (**mOutputCurr)(0,0), (**mOutputCurr)(1,0));
}

Task::List PLL::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
