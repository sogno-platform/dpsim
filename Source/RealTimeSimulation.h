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

#pragma once

#include <signal.h>

#include "Config.h"
#include "Simulation.h"

#define RTMETHOD_TIMERFD
//#define RTMETHOD_EXCEPTIONS

namespace DPsim {

#ifdef RTMETHOD_EXCEPTIONS
	class TimerExpiredException {
	};
#endif

	class RealTimeSimulation : public Simulation {

	protected:
#ifdef RTMETHOD_EXCEPTIONS
		static void alarmHandler(int, siginfo_t*, void*);
		timer_t mTimer;
		sigset_t alrmset;
		uint64_t mTimerCount = 0;
#elif defined(RTMETHOD_TIMERFD)
		int mTimerFd;
#else
  #error Unkown real-time execution mode
#endif

		void startTimer();
		void stopTimer();

	public:
		RealTimeSimulation(String name, Component::List comps, Real om, Real dt, Real tf, Logger::Level logLevel = Logger::Level::INFO, SimulationType domain = SimulationType::DP, Int downSampleRate = 1);
		~RealTimeSimulation();

		/* Perform the main simulation loop in real time.
		 *
		 * @param startSynch If true, the simulation waits for the first external value before starting the timing.
		 */
		void run(double duration, bool startSynch = true);

		void run(bool startSynch = true);
	};
}
