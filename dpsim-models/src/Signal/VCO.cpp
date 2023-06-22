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
    mInputRef(Attribute<Real>::createDynamic("input_ref", mAttributes)),
    /// CHECK: Which of these really need to be attributes?
    mInputPrev(Attribute<Matrix>::create("input_prev", mAttributes, Matrix::Zero(2,1))),
    mStatePrev(Attribute<Matrix>::create("state_prev", mAttributes, Matrix::Zero(2,1))),
    mOutputPrev(Attribute<Matrix>::create("output_prev", mAttributes, Matrix::Zero(2,1))),
    mInputCurr(Attribute<Matrix>::create("input_curr", mAttributes, Matrix::Zero(2,1))),
    mStateCurr(Attribute<Matrix>::create("state_curr", mAttributes, Matrix::Zero(2,1))),
    mOutputCurr(Attribute<Matrix>::create("output_curr", mAttributes, Matrix::Zero(2,1))) { }


void VCO::setParameters(Real omegaNom) {
    mOmegaNom = omegaNom;
    mSLog->info("OmegaNom = {}", mOmegaNom);

    // First entry of input vector is constant omega
    (**mInputCurr)(0,0) = mOmegaNom;
}

void VCO::setSimulationParameters(Real timestep) {
    mTimeStep = timestep;
    mSLog->info("Integration step = {}", mTimeStep);
}

void VCO::setInitialValues(Real input_init, Matrix state_init, Matrix output_init) {
	(**mInputCurr)(1,0) = input_init;
    **mStateCurr = state_init;
    **mOutputCurr = output_init;

    mSLog->info("Initial values:");
    mSLog->info("inputCurrInit = ({}, {}), stateCurrInit = ({}, {}), outputCurrInit = ({}, {})", (**mInputCurr)(0,0), (**mInputCurr)(1,0), (**mInputPrev)(0,0), (**mInputPrev)(1,0), (**mStateCurr)(0,0), (**mStateCurr)(1,0), (**mStatePrev)(0,0), (**mStatePrev)(1,0));
}

void VCO::composeStateSpaceMatrices() {
    mA <<   0,  0,
            0,  0;
    mB <<   1,  0,
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
    (**mInputCurr)(1,0) = **mInputRef;

    mSLog->info("Time {}:", time);
    mSLog->info("Input values: inputCurr = ({}, {}), inputPrev = ({}, {}), stateCurr = ({}, {}), statePrev = ({}, {})", (**mInputCurr)(0,0), (**mInputCurr)(1,0), (**mInputPrev)(0,0), (**mInputPrev)(1,0), (**mStateCurr)(0,0), (**mStateCurr)(1,0), (**mStatePrev)(0,0), (**mStatePrev)(1,0));

    **mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
    **mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;

    mSLog->info("State values: stateCurr = ({}, {})", (**mStateCurr)(0,0), (**mStateCurr)(1,0));
    mSLog->info("Output values: outputCurr = ({}, {}):", (**mOutputCurr)(0,0), (**mOutputCurr)(1,0));
}

Task::List VCO::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}