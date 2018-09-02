/** Timer for real-time execution
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/Config.h>

#include <chrono>

namespace DPsim {

	class Timer {
	public:
		struct OverrunException {
			uint64_t overruns;
		};

		using Ticks = std::chrono::nanoseconds;

		using StartClock = std::chrono::system_clock;
		using IntervalClock = std::chrono::steady_clock;

		using StartTimePoint = std::chrono::time_point<StartClock, Ticks>;
		using IntervalTimePoint = std::chrono::time_point<IntervalClock, Ticks>;

	protected:
		enum State {
			running,
			stopped
		} mState;

		StartTimePoint mStartAt;
		IntervalTimePoint mNextTick;
		Ticks mTickInterval;

#ifdef HAVE_TIMERFD
		int mTimerFd;
#endif
		int mOverruns;
		int mTicks;
		int mFlags;

	public:
		enum Flags : int {
			fail_on_overrun = 1
		};

		Timer(int flags = 0);

		~Timer();

		void start(const StartTimePoint &startAt);

		/// Start real-time timer
		void start();

		/// Stop real-time timer
		void stop();

		/// Suspend thread execution until next tick
		void sleep();

		// Getter
		int overruns() {
			return mOverruns;
		}

		int ticks() {
			return mTicks;
		}

		// Setter
		void setStartTime(const StartTimePoint &start) {
			mStartAt = start;
		}

		void setInterval(const Ticks &intv) {
			mTickInterval = intv;
		}
};

template<class Rep, class Period>
struct timespec to_timespec(std::chrono::duration<Rep, Period> dur) {
	auto nsDur = std::chrono::duration_cast<std::chrono::nanoseconds>(dur);

	struct timespec ts;

	ts.tv_sec  = nsDur.count() / 1000000000;
	ts.tv_nsec = nsDur.count() % 1000000000;

	return ts;
}

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

template<typename Rep, typename Period>
std::ostream &operator<<(std::ostream &stream,
	const std::chrono::duration<Rep, Period> &dur) {

	using namespace std::chrono;

	auto d = dur;

	if (dur == dur.zero())
		stream << "0";
	else {
		auto h = duration_cast<hours>(d);
		d -= h;
		auto m = duration_cast<minutes>(d);
		d -= m;
		auto s = duration_cast<seconds>(d);
		d -= s;
		auto ms = duration_cast<milliseconds>(d);
		d -= ms;
		auto us = duration_cast<microseconds>(d);
		d -= us;
		auto ns = duration_cast<nanoseconds>(d);

		long int vals[] = { h.count(), m.count(), s.count(), ms.count(), us.count(), ns.count() };
		const char *units[] = { "hrs", "mins", "secs", "msecs", "usecs", "nsecs" };

		for (int i = 0; i < 6; i++) {
			if (i)
				stream << " ";

			if (vals[i] == 0)
				continue;

			stream << vals[i] << " " << units[i];
		}
	}

	return stream;
}
