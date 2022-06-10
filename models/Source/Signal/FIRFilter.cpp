/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/FIRFilter.h>

using namespace CPS;
using namespace CPS::Signal;

FIRFilter::FIRFilter(String uid, String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
	mCurrentIdx(0),
	mOutput(Attribute<Real>::create("output", mAttributes)),
	mInitSample(Attribute<Real>::create("init_sample", mAttributes, 0.0)) {
}

FIRFilter::FIRFilter(String name, std::vector<Real> filterCoefficients, Real initSample, Logger::Level logLevel)
	: FIRFilter(name, name, logLevel) {

	mFilter = filterCoefficients;
	mFilterLength = (Int) mFilter.size();
}

void FIRFilter::initialize(Real timeStep) {
	mSignal.assign(mFilterLength, **mInitSample);
	mSLog->info("Initialize filter with {}", **mInitSample);
}

void FIRFilter::step(Real time) {
	Real output = 0;
	///CHECK: Does this copy the value or the reference of the data behind mInput? It should only copy the value.
	mSignal[mCurrentIdx] = **mInput;
	for (int i = 0; i < mFilterLength; i++) {
		output += mFilter[i] * mSignal[getIndex(i)];
	}

	incrementIndex();
	**mOutput = output;
	SPDLOG_LOGGER_DEBUG(mSLog, "Set output to {}", output);
}

void FIRFilter::Step::execute(Real time, Int timeStepCount) {
	mFilter.step(time);
}

Task::List FIRFilter::getTasks() {
	return Task::List({std::make_shared<FIRFilter::Step>(*this)});
}

void FIRFilter::incrementIndex () {
	mCurrentIdx = (mCurrentIdx + 1) % mFilterLength;
}

Int FIRFilter::getIndex(Int index) {
	return (mCurrentIdx + index) % mFilterLength;
}

void FIRFilter::setInput(Attribute<Real>::Ptr input) {
	mInput = input;
}
