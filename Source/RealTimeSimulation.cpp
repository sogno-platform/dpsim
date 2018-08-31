/** Simulation
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include <signal.h>
#include <time.h>
#include <unistd.h>

#ifdef HAVE_TIMERFD
  #include <sys/timerfd.h>
#endif /* HAVE_TIMERFD */

using namespace CPS;
using namespace DPsim;

RealTimeSimulation::RealTimeSimulation(String name, SystemTopology system, Real timeStep, Real finalTime,
		Domain domain, Solver::Type type, Logger::Level logLevel, Bool steadyStateInit)
	: Simulation(name, system, timeStep, finalTime, domain, type, logLevel, steadyStateInit),
	mTimeStep(timeStep),
	mOverruns(0)
{
	mAttributes["time_step"] = Attribute<Real>::make(&mTimeStep, Flags::read);
	mAttributes["overruns"] = Attribute<Int>::make(&mOverruns, Flags::read);

	mTimerFd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (mTimerFd < 0) {
		throw SystemError("Failed to create timerfd");
	}
}


RealTimeSimulation::~RealTimeSimulation()
{
	close(mTimerFd);
}

void RealTimeSimulation::startTimer(const StartClock::time_point &startAt)
{
	/* Determine offset between clocks */
	auto rt     = std::chrono::system_clock::now();
	auto steady = std::chrono::steady_clock::now();

	struct itimerspec ts;

	/* This handles the offset between
	 * - CLOCK_MONOTONIC (aka std::chrono::steady_clock) and
	 * - CLOCK_REALTIME (aka std::chrono::system_clock)
	 */
	auto startAtDur = startAt.time_since_epoch() - rt.time_since_epoch() + steady.time_since_epoch();
	auto startAtNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(startAtDur);

	ts.it_value.tv_sec  = startAtNSecs.count() / 1000000000;
	ts.it_value.tv_nsec = startAtNSecs.count() % 1000000000;
	ts.it_interval.tv_sec  = (time_t) mTimeStep;
	ts.it_interval.tv_nsec = (long) (mTimeStep * 1e9);

	if (timerfd_settime(mTimerFd, TFD_TIMER_ABSTIME, &ts, 0) < 0) {
		throw SystemError("Failed to arm timerfd");
	}
}

void RealTimeSimulation::stopTimer()
{
	struct itimerspec ts;

	ts.it_value = { 0, 0 };

	if (timerfd_settime(mTimerFd, 0, &ts, 0) < 0) {
		throw SystemError("Failed to arm timerfd");
	}
}

void RealTimeSimulation::run(const StartClock::duration &startIn)
{
	run(StartClock::now() + startIn);
}

void RealTimeSimulation::run(const StartClock::time_point &startAt)
{
	auto startAtDur = startAt.time_since_epoch();
	auto startAtNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(startAtDur);

#ifdef WITH_SHMEM
	// We send initial state over all interfaces
	for (auto ifm : mInterfaces) {
		ifm.interface->writeValues();
	}

	std::cout << Logger::prefix() << "Waiting for start synchronization on " << mInterfaces.size() << " interfaces" << std::endl;

	// Blocking wait for interfaces
	for (auto ifm : mInterfaces) {
		ifm.interface->readValues(ifm.syncStart);
	}

	std::cout << Logger::prefix() << "Synchronized simulation start with remotes" << std::endl;
#endif

	std::cout << Logger::prefix() << "Starting simulation at " << startAt << " (delta_T = " << std::chrono::duration_cast<std::chrono::seconds>(startAt - StartClock::now()).count() << " seconds)" << std::endl;

	startTimer(startAt);

	for (auto ifm : mInterfaces) {
		ifm.interface->writeValues();
	}

	// main loop
	Int steps = 0;
	do {
		uint64_t overrun;
		step();

		if (read(mTimerFd, &overrun, sizeof(overrun)) < 0) {
			throw SystemError("Read from timerfd failed");
		}
		if (overrun > 1) {
			mLog.Log(Logger::Level::WARN) << "Overrun of "<< overrun-1 << " timesteps at " << mTime << std::endl;
			mOverruns =+ overrun - 1;
		}

		if (steps++ == 0)
			std::cout << Logger::prefix() << "Simulation started!" << std::endl;

	} while (mTime < mFinalTime);

	stopTimer();
}
