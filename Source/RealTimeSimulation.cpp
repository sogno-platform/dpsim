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

void RealTimeSimulation::alarmHandler(int sig, siginfo_t* si, void* ctx)
{
	auto sim = static_cast<RealTimeSimulation*>(si->si_value.sival_ptr);

	/* only throw an exception if we're actually behind */
	if (++sim->mTimerCount * sim->mSystemModel.getTimeStep() > sim->mTime)
		throw TimerExpiredException();
}

void RealTimeSimulation::run(Method method, bool startSynch)
{
	char timebuf[8];
	int ret, sig, timerfd;
	sigset_t alrmset;
	struct sigaction sa;
	struct sigevent evp;
	struct itimerspec ts;
	timer_t timer;
	uint64_t overrun;

	// initialize timer / timerfd
	if (method == Method::TimerFD) {
		timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
		if (timerfd < 0) {
			std::perror("Failed to create timerfd");
			std::exit(1);
		}
	}
	else if (method == Method::Exceptions) {
		sa.sa_sigaction = RealTimeSimulation::alarmHandler;
		sa.sa_flags = SA_SIGINFO;
		sigemptyset(&sa.sa_mask);
		if (sigaction(SIGALRM, &sa, NULL)) {
			std::perror("Failed to establish SIGALRM handler");
			std::exit(1);
		}

		evp.sigev_notify = SIGEV_SIGNAL;
		evp.sigev_signo = SIGALRM;
		evp.sigev_value.sival_ptr = this;
		if (timer_create(CLOCK_MONOTONIC, &evp, &timer)) {
			std::perror("Failed to create timer");
			std::exit(1);
		}

		sigemptyset(&alrmset);
		sigaddset(&alrmset, SIGALRM);
	}
	else {
		std::cerr << "invalid rt method, exiting" << std::endl;
		std::exit(1);
	}

	ts.it_value.tv_sec = (time_t) mSystemModel.getTimeStep();
	ts.it_value.tv_nsec = (long) (mSystemModel.getTimeStep() * 1e9);
	ts.it_interval = ts.it_value;

	// optional start synchronization
	if (startSynch) {
		step(false); // first step, sending the initial values
		step(true); // blocking step for synchronization + receiving the initial state of the other network
		increaseByTimeStep();
	}

	// arm timer
	if (method == Method::TimerFD) {
		if (timerfd_settime(timerfd, 0, &ts, 0) < 0) {
			std::perror("Failed to arm timerfd");
			std::exit(1);
		}
	}
	else if (method == Method::Exceptions) {
		if (timer_settime(timer, 0, &ts, NULL)) {
			std::perror("Failed to arm timer");
			std::exit(1);
		}
	}

	// main loop
	do {
		if (method == Method::Exceptions) {
			try {
				ret = step(false);
				sigwait(&alrmset, &sig);
			}
			catch (TimerExpiredException& e) {
				std::cerr << "timestep expired at " << mTime << std::endl;
			}
		}
		else if (method == Method::TimerFD) {
			ret = step(false);
			if (read(timerfd, timebuf, 8) < 0) {
				std::perror("Read from timerfd failed");
				std::exit(1);
			}
			overrun = *((uint64_t*) timebuf);
			if (overrun > 1) {
				std::cerr << "timerfd overrun of " << overrun-1 << " at " << mTime << std::endl;
			}
		}
		increaseByTimeStep();
		if (!ret)
			break;
	} while (ret);

	// cleanup
	if (method == Method::TimerFD) {
		close(timerfd);
	}
	else if (method == Method::Exceptions) {
		timer_delete(timer);
	}
}
