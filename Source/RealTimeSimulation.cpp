/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <chrono>
#include <ctime>
#include <dpsim/RealTimeSimulation.h>
#include <iomanip>

using namespace CPS;
using namespace DPsim;

RealTimeSimulation::RealTimeSimulation(String name, Logger::Level logLevel)
	: Simulation(name, logLevel), mTimer() {

	addAttribute<Int >("overruns", nullptr, [=](){ return mTimer.overruns(); }, Flags::read);
	//addAttribute<Int >("overruns", nullptr, nullptr, Flags::read);
}

void RealTimeSimulation::run(const Timer::StartClock::duration &startIn) {
	run(Timer::StartClock::now() + startIn);
}

void RealTimeSimulation::run(const Timer::StartClock::time_point &startAt) {
	if (!mInitialized)
		initialize();

#ifdef WITH_SHMEM
	mLog->info("Opening interfaces.");

	for (auto ifm : mInterfaces)
		ifm.interface->open(mLog);

	sync();
#endif

	auto now_time = std::chrono::system_clock::to_time_t(startAt);
	mLog->info("Starting simulation at {} (delta_T = {} seconds)",
			  std::put_time(std::localtime(&now_time), "%F %T"),
			  std::chrono::duration_cast<std::chrono::seconds>(startAt - Timer::StartClock::now()).count());

	mTimer.setStartTime(startAt);
	mTimer.setInterval(mTimeStep);
	mTimer.start();

	// main loop
	do {
		mTimer.sleep();
		step();

		if (mTimer.ticks() == 1)
			mLog->info("Simulation started.");
	} while (mTime < mFinalTime);

	mLog->info("Simulation finished.");

	mScheduler->stop();

#ifdef WITH_SHMEM
	for (auto ifm : mInterfaces)
		ifm.interface->close();
#endif

	for (auto lg : mLoggers)
		lg->close();

	mTimer.stop();
}
