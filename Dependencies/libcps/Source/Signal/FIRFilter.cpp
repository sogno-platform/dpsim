/**
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/Signal/FIRFilter.h>

using namespace CPS;
using namespace CPS::Signal;

FIRFilter::FIRFilter(String uid, String name, Logger::Level logLevel) :
	SignalComponent(name, name, logLevel),
	mCurrentIdx(0),
	mInitSample(0.0) {

	addAttribute<Real>("output", &mOutput, Flags::read);
	addAttribute<Real>("init_sample", &mInitSample, Flags::write | Flags::read);
}

FIRFilter::FIRFilter(String name, std::vector<Real> filterCoefficients, Real initSample, Logger::Level logLevel)
	: FIRFilter(name, name, logLevel) {

	mFilter = filterCoefficients;
	mFilterLength = (Int) mFilter.size();
}

void FIRFilter::initialize(Real timeStep) {
	mSignal.assign(mFilterLength, mInitSample);
	mSLog->info("Initialize filter with {}", mInitSample);
}

void FIRFilter::step(Real time) {
	Real output = 0;
	mSignal[mCurrentIdx] = mInput->getByValue();
	for (int i = 0; i < mFilterLength; i++) {
		output += mFilter[i] * mSignal[getIndex(i)];
	}

	incrementIndex();
	mOutput = output;
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
