/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <chrono>
#include <ctime>
#include <dpsim/RealTimeSimulation.h>
#include <iomanip>
#include <fmt/chrono.h>

using namespace CPS;
using namespace DPsim;

RealTimeSimulation::RealTimeSimulation(String name, CommandLineArgs &args)
    : Simulation(name, args), mTimer(){};

RealTimeSimulation::RealTimeSimulation(String name, Logger::Level logLevel)
    : Simulation(name, logLevel), mTimer() {

  //addAttribute<Int >("overruns", nullptr, [=](){ return mTimer.overruns(); }, Flags::read);
  //addAttribute<Int >("overruns", nullptr, nullptr, Flags::read);
}

void RealTimeSimulation::run(const Timer::StartClock::duration &startIn) {
  run(Timer::StartClock::now() + startIn);
}

void RealTimeSimulation::run(const Timer::StartClock::time_point &startAt) {
  if (!mInitialized)
    initialize();

  for (auto lg : mLoggers)
    lg->start();

  SPDLOG_LOGGER_INFO(mLog, "Opening interfaces.");

  for (auto intf : mInterfaces)
    intf->open();

  sync();

  SPDLOG_LOGGER_INFO(mLog, "Starting simulation at {:%Y-%m-%d %H:%M:%S} (delta_T = {} seconds)",
                     startAt, startAt - Timer::StartClock::now());

  mTimer.setStartTime(startAt);
  mTimer.setInterval(**mTimeStep);
  mTimer.start();

  // main loop
  do {
    mTimer.sleep();
    step();

    if (mTimer.ticks() == 1)
      SPDLOG_LOGGER_INFO(mLog, "Simulation started.");
  } while (mTime < **mFinalTime);

  SPDLOG_LOGGER_INFO(mLog, "Simulation finished.");

  mScheduler->stop();

  for (auto intf : mInterfaces)
    intf->close();

  for (auto lg : mLoggers)
    lg->stop();

  mTimer.stop();
}
