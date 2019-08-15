/** Simulation
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
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

#include <dpsim/RealTimeSimulation.h>

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
	initialize();

#ifdef WITH_SHMEM
	std::cout << Logger::prefix() << "Opening interfaces." << std::endl;

	for (auto ifm : mInterfaces)
		ifm.interface->open();

	sync();
#endif

	std::cout << Logger::prefix() << "Start simulation at " << startAt
		<< " (delta_T = " << startAt - Timer::StartClock::now() << ")." << std::endl;

	mTimer.setStartTime(startAt);
	mTimer.setInterval(mTimeStep);
	mTimer.start();

	// main loop
	do {
		mTimer.sleep();
		step();

		if (mTimer.ticks() == 1)
			std::cout << Logger::prefix() << "Simulation started." << std::endl;
	} while (mTime < mFinalTime);

	std::cout << Logger::prefix() << "Simulation finished." << std::endl;

	mScheduler->stop();

#ifdef WITH_SHMEM
	for (auto ifm : mInterfaces)
		ifm.interface->close();
#endif

	for (auto lg : mLoggers)
		lg->close();

	mTimer.stop();
}
