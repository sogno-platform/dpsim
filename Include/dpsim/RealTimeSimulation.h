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

#include <chrono>

#include "Config.h"
#include "Simulation.h"

#define RTMETHOD_TIMERFD
//#define RTMETHOD_EXCEPTIONS

using namespace CPS;

namespace DPsim {

#ifdef RTMETHOD_EXCEPTIONS
	class TimerExpiredException {
	};
#endif

	class RealTimeSimulation : public Simulation {

	protected:
		typedef std::chrono::system_clock StartClock;

		// TODO: we should use a std::chrono::duration here!
		Real mTimeStep;
		Int mOverruns;

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

		/** Start the real-time timer.
		 *
		 * If no start time is provided, we will default to now() + mTimeStep
		 */
		void startTimer(const StartClock::time_point &startAt);
		/** Stop the real-time timer. */
		void stopTimer();

		void createTimer();
		void destroyTimer();

	public:
		RealTimeSimulation(String name, SystemTopology system, Real timeStep, Real finalTime,
			Solver::Domain domain = Solver::Domain::DP, Solver::Type type = Solver::Type::MNA,
			Logger::Level logLevel = Logger::Level::INFO, Bool steadyStateInit = false);
		~RealTimeSimulation();

		/** Perform the main simulation loop in real time.
		 *
		 * @param startSynch If true, the simulation waits for the first external value before starting the timing.
		 */
		void run(bool startSynch = true, const StartClock::duration &startIn = std::chrono::milliseconds(100));

		void run(bool startSynch, const StartClock::time_point &startAt);
	};
}

#include <iomanip>
#include <iostream>
#include <ctime>

template<typename Clock, typename Duration>
std::ostream &operator<<(std::ostream &stream,
	const std::chrono::time_point<Clock, Duration> &time_point) {
	const time_t time = Clock::to_time_t(time_point);
#if __GNUC__ > 4 || ((__GNUC__ == 4) && __GNUC_MINOR__ > 8 && __GNUC_REVISION__ > 1)
	// Maybe the put_time will be implemented later?
	struct tm tm;
	localtime_r(&time, &tm);
	return stream << std::put_time(&tm, "%c"); // Print standard date&time
#else
	char buffer[26];
	ctime_r(&time, buffer);
	buffer[24] = '\0';  // Removes the newline that is added
	return stream << buffer;
#endif
}
