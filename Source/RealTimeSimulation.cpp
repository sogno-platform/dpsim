/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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

RealTimeSimulation::RealTimeSimulation(String name, SystemTopology system, Real timeStep, Real finalTime,
		Domain domain, Solver::Type type, Logger::Level logLevel, Bool steadyStateInit)
	: Simulation(name, system, timeStep, finalTime, domain, type, logLevel, steadyStateInit),
	mTimer() {

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
