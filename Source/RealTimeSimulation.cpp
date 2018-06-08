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

#include "RealTimeSimulation.h"

#include <signal.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

using namespace CPS;
using namespace DPsim;

RealTimeSimulation::RealTimeSimulation(String name, SystemTopology system, Real timeStep, Real finalTime,
		Solver::Domain domain, Solver::Type type, Logger::Level logLevel, Bool steadyStateInit)
	: Simulation(name, system, timeStep, finalTime, domain, type, logLevel, steadyStateInit),
	mTimeStep(timeStep),
	mOverruns(0)
{
	mAttributes["time_step"] = Attribute<Real>::make(&mTimeStep, Flags::read);
	mAttributes["overruns"] = Attribute<Int>::make(&mOverruns, Flags::read);

	createTimer();
}


RealTimeSimulation::~RealTimeSimulation()
{
	destroyTimer();
}

#ifdef RTMETHOD_EXCEPTIONS
void RealTimeSimulation::alarmHandler(int sig, siginfo_t* si, void* ctx)
{
	auto sim = static_cast<RealTimeSimulation*>(si->si_value.sival_ptr);

	/* only throw an exception if we're actually behind */
	if (++sim->mTimerCount * sim->mTimeStep > sim->mTime)
		throw TimerExpiredException();
}
#endif

void RealTimeSimulation::createTimer()
{
#ifdef RTMETHOD_TIMERFD
	mTimerFd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (mTimerFd < 0) {
		throw SystemError("Failed to create timerfd");
	}
#elif defined(RTMETHOD_EXCEPTIONS)
	struct sigaction sa;
	struct sigevent evp;

	sa.sa_sigaction = RealTimeSimulation::alarmHandler;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);

	if (sigaction(SIGALRM, &sa, NULL)) {
		throw SystemError("Failed to establish SIGALRM handler");
	}

	evp.sigev_notify = SIGEV_SIGNAL;
	evp.sigev_signo = SIGALRM;
	evp.sigev_value.sival_ptr = this;

	if (timer_create(CLOCK_MONOTONIC, &evp, &timer)) {
		throw SystemError("Failed to create timerfd");
	}

	sigemptyset(&mAlrmset);
	sigaddset(&mAlrmset, SIGALRM);
#else
  #error Unkown real-time execution mode
#endif
}

void RealTimeSimulation::destroyTimer()
{
#ifdef RTMETHOD_TIMERFD
	close(mTimerFd);
#elif defined(RTMETHOD_EXCEPTIONS)
	timer_delete(mTimer);
#else
  #error Unkown real-time execution mode
#endif
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

#ifdef RTMETHOD_TIMERFD
	if (timerfd_settime(mTimerFd, TFD_TIMER_ABSTIME, &ts, 0) < 0) {
		throw SystemError("Failed to arm timerfd");
	}
#elif defined(RTMETHOD_EXCEPTIONS)
	if (timer_settime(mTimer, TIMER_ABSTIME, &ts, NULL)) {
		throw SystemError("Failed to arm timer");
	}
#else
  #error Unkown real-time execution mode
#endif
}

void RealTimeSimulation::stopTimer()
{
	struct itimerspec ts;

	ts.it_value = { 0, 0 };

#ifdef RTMETHOD_TIMERFD
	if (timerfd_settime(mTimerFd, 0, &ts, 0) < 0) {
		throw SystemError("Failed to arm timerfd");
	}
#elif defined(RTMETHOD_EXCEPTIONS)
	if (timer_settime(mTimer, 0, &ts, NULL)) {
		throw SystemError("Failed to arm timer");
	}
#else
  #error Unkown real-time execution mode
#endif
}

void RealTimeSimulation::run(bool startSynch, const StartClock::duration &startIn)
{
	run(startSynch, StartClock::now() + startIn);
}

void RealTimeSimulation::run(bool startSynch, const StartClock::time_point &startAt)
{
	auto startAtDur = startAt.time_since_epoch();
	auto startAtNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(startAtDur);

	if (startSynch) {
		mTime = 0;

		step(false); // first step, sending the initial values
		step(true); // blocking step for synchronization + receiving the initial state of the other network

		std::cout << "Synchronized simulation start with remotes" << std::endl;
	}

	std::cout << "Starting simulation at " << startAt << " (delta_T = " << std::chrono::duration_cast<std::chrono::seconds>(startAt - StartClock::now()).count() << " seconds)" << std::endl;

	startTimer(startAt);

	// main loop
	Int steps = 0;
	do {
#ifdef RTMETHOD_TIMERFD
		uint64_t overrun;
		step(false);

		if (read(mTimerFd, &overrun, sizeof(overrun)) < 0) {
			throw SystemError("Read from timerfd failed");
		}
		if (overrun > 1) {
			mLog.Log(Logger::Level::WARN) << "Overrun of "<< overrun-1 << " timesteps at " << mTime << std::endl;
			mOverruns =+ overrun - 1;
		}
#elif defined(RTMETHOD_EXCEPTIONS)
		try {
			step(false);
			sigwait(&alrmset, &sig);
		}
		catch (TimerExpiredException& e) {
			// TODO: TimerExpired does not actually indicate an overrun
			//       It rather signals that the timestep is coming to and end.
			//mLog.Log(Logger::Level::WARN) << "Overrun at " << mTime << std::endl;
			//mOverruns = timer_getoverrun(mTimer);
		}
#else
  #error Unkown real-time execution mode
#endif
		if (steps++ == 0)
			std::cout << "Simulation started!" << std::endl;

	} while (mTime < mFinalTime);

	stopTimer();
}
