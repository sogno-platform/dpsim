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

using namespace DPsim;

RealTimeSimulation::RealTimeSimulation(String name, Component::List comps, Real om, Real dt, Real tf, Logger::Level logLevel, SimulationType simType, Int downSampleRate)
	: Simulation(name, comps, om, dt, tf, logLevel, simType, downSampleRate)
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

RealTimeSimulation::~RealTimeSimulation()
{
#ifdef RTMETHOD_TIMERFD
	close(mTimerFd);
#elif defined(RTMETHOD_EXCEPTIONS)
	timer_delete(mTimer);
#else
  #error Unkown real-time execution mode
#endif
}

#ifdef RTMETHOD_EXCEPTIONS
void RealTimeSimulation::alarmHandler(int sig, siginfo_t* si, void* ctx)
{
	auto sim = static_cast<RealTimeSimulation*>(si->si_value.sival_ptr);

	/* only throw an exception if we're actually behind */
	if (++sim->mTimerCount * sim->mSystemModel.getTimeStep() > sim->mTime)
		throw TimerExpiredException();
}
#endif

void RealTimeSimulation::startTimer()
{
	struct itimerspec ts;

	ts.it_value.tv_sec = (time_t) mSystemModel.getTimeStep();
	ts.it_value.tv_nsec = (long) (mSystemModel.getTimeStep() * 1e9);
	ts.it_interval = ts.it_value;

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

void RealTimeSimulation::run(bool startSynch)
{
	run(mFinalTime - mTime, startSynch);
}

void RealTimeSimulation::run(double duration, bool startSynch)
{
	int ret;
	uint64_t overrun;

	if (startSynch) {
		step(false); // first step, sending the initial values
		step(true); // blocking step for synchronization + receiving the initial state of the other network
		increaseByTimeStep();
	}

	double started = mTime;

	startTimer();

	// main loop
	do {
#ifdef RTMETHOD_TIMERFD
		ret = step(false);

		if (read(mTimerFd, &overrun, sizeof(overrun)) < 0) {
			throw SystemError("Read from timerfd failed");
		}
		if (overrun > 1) {
			mLog.Log(Logger::Level::WARN) << "Overrun of "<< overrun-1 << " timesteps at " << mTime << std::endl;
		}
#elif defined(RTMETHOD_EXCEPTIONS)
		try {
			ret = step(false);
			sigwait(&alrmset, &sig);
		}
		catch (TimerExpiredException& e) {
			mLog.Log(Logger::Level::WARN) << "Overrun at " << mTime << std::endl;
		}
#else
  #error Unkown real-time execution mode
#endif
		increaseByTimeStep();

	} while (ret && mTime - started < duration);

	stopTimer();
}
