/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/Integrator.h>

using namespace CPS;
using namespace CPS::Signal;

Integrator::Integrator(String name, Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mInputRef(mAttributes->createDynamic<Real>("input_ref")),
      /// CHECK: Which of these really need to be attributes?
      mInputPrev(mAttributes->create<Real>("input_prev")),
      mStatePrev(mAttributes->create<Real>("state_prev")),
      mOutputPrev(mAttributes->create<Real>("output_prev")),
      mInputCurr(mAttributes->create<Real>("input_curr")),
      mStateCurr(mAttributes->create<Real>("state_curr")),
      mOutputCurr(mAttributes->create<Real>("output_curr")) {}

void Integrator::setParameters(Real timestep) {
  mTimeStep = timestep;

  SPDLOG_LOGGER_INFO(mSLog, "Integration step = {}", mTimeStep);
}

void Integrator::setInitialValues(Real input_init, Real state_init,
                                  Real output_init) {
  **mInputCurr = input_init;
  **mStateCurr = state_init;
  **mOutputCurr = output_init;

  SPDLOG_LOGGER_INFO(mSLog, "Initial values:");
  SPDLOG_LOGGER_INFO(
      mSLog, "inputCurrInit = {}, stateCurrInit = {}, outputCurrInit = {}",
      **mInputCurr, **mStateCurr, **mOutputCurr);
}

void Integrator::signalAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mInputCurr);
  prevStepDependencies.push_back(mOutputCurr);
  modifiedAttributes.push_back(mInputPrev);
  modifiedAttributes.push_back(mOutputPrev);
};

void Integrator::signalPreStep(Real time, Int timeStepCount) {
  **mInputPrev = **mInputCurr;
  **mStatePrev = **mStateCurr;
  **mOutputPrev = **mOutputCurr;
}

void Integrator::signalAddStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  attributeDependencies.push_back(mInputRef);
  modifiedAttributes.push_back(mInputCurr);
  modifiedAttributes.push_back(mOutputCurr);
};

void Integrator::signalStep(Real time, Int timeStepCount) {
  **mInputCurr = **mInputRef;

  SPDLOG_LOGGER_INFO(mSLog, "Time {}:", time);
  SPDLOG_LOGGER_INFO(
      mSLog, "Input values: inputCurr = {}, inputPrev = {}, statePrev = {}",
      **mInputCurr, **mInputPrev, **mStatePrev);

  **mStateCurr = **mStatePrev + mTimeStep / 2.0 * **mInputCurr +
                 mTimeStep / 2.0 * **mInputPrev;
  **mOutputCurr = **mStateCurr;

  SPDLOG_LOGGER_INFO(mSLog, "State values: stateCurr = {}", **mStateCurr);
  SPDLOG_LOGGER_INFO(mSLog, "Output values: outputCurr = {}:", **mOutputCurr);
}

Task::List Integrator::getTasks() {
  return Task::List(
      {std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
