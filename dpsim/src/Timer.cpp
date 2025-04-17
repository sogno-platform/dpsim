/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <assert.h>
#include <thread>

#include <dpsim-models/Definitions.h>
#include <dpsim/Timer.h>

#ifdef HAVE_TIMERFD
#include <sys/timerfd.h>
#include <unistd.h>
#endif // HAVE_TIMERFD

using namespace DPsim;
using CPS::SystemError;

Timer::Timer(int flags)
    : mState(stopped), mOverruns(0), mTicks(0), mFlags(flags) {
#ifdef HAVE_TIMERFD
  mTimerFd = timerfd_create(CLOCK_MONOTONIC, 0);
  if (mTimerFd < 0) {
    throw SystemError("Failed to create timerfd");
  }
#else
  std::cerr << "WARNING: No high resolution timer available. Clock might drift!"
            << std::endl;
#endif
}

Timer::~Timer() {
  if (mState == State::running)
    try {
      stop();
    } catch (SystemError &e) {
      std::cerr << "ERROR: The timer was not stopped properly: " << std::endl;
    }

#ifdef HAVE_TIMERFD
  close(mTimerFd);
#endif
}

void Timer::sleep() {
  uint64_t ticks = 0, overruns;

#ifdef HAVE_TIMERFD
  ssize_t bytes;

  bytes = read(mTimerFd, &ticks, sizeof(ticks));
  if (bytes < 0) {
    throw SystemError("Read from timerfd failed");
  }
#else
  std::this_thread::sleep_until(mNextTick);

  auto now = IntervalClock::now();

  while (mNextTick < now) {
    mNextTick += mTickInterval;
    ticks++;
  }

#endif
  overruns = ticks - 1;

  mOverruns += overruns;
  mTicks += ticks;

  if (overruns > 0) {
    //SPDLOG_LOGGER_WARN(mSLog, "Timer overrun of {} timesteps at {}", overruns, mTime);
    if (mFlags & Flags::fail_on_overrun)
      throw OverrunException{overruns};
  }
}

void Timer::start() {
  assert(mState == stopped);

  mTicks = 0;
  mOverruns = 0;

  // Determine offset between clocks.
  auto rt = StartClock::now();
  auto steady = IntervalClock::now();

  /* This handles the offset between
   * - IntervalClock (CLOCK_MONOTONIC aka std::chrono::steady_clock) and
   * - StartClock (CLOCK_REALTIME aka std::chrono::system_clock)
   */
  auto start = mStartAt > StartTimePoint()
                   ? mStartAt.time_since_epoch() - rt.time_since_epoch() +
                         steady.time_since_epoch()
                   : steady.time_since_epoch();

#ifdef HAVE_TIMERFD
  int ret;
  struct itimerspec ts = {.it_interval = to_timespec(mTickInterval),
                          .it_value = to_timespec(start)};

  ret = timerfd_settime(mTimerFd, TFD_TIMER_ABSTIME, &ts, 0);
  if (ret < 0) {
    throw SystemError("Failed to arm timerfd");
  }
#endif
  mNextTick = IntervalTimePoint(start) + mTickInterval;
  mState = State::running;
}

void Timer::stop() {
  assert(mState == State::running);

#ifdef HAVE_TIMERFD
  int ret;
  struct itimerspec ts = {.it_interval = {0, 0}, .it_value = {0, 0}};

  ret = timerfd_settime(mTimerFd, 0, &ts, 0);
  if (ret < 0) {
    throw SystemError("Failed to arm timerfd");
  }
#endif

  mState = State::stopped;
}
