/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <signal.h>

#include <chrono>

#include <dpsim/Config.h>
#include <dpsim/Simulation.h>
#include <dpsim/Timer.h>

namespace DPsim {
/// Extending Simulation class by real-time functionality.
class RealTimeSimulation : public Simulation {

protected:
  Timer mTimer;

public:
  /// Standard constructor
  RealTimeSimulation(String name,
                     CPS::Logger::Level logLevel = CPS::Logger::Level::info);

  /** Perform the main simulation loop in real time.
		 *
		 * @param startSynch If true, the simulation waits for the first external value before starting the timing.
		 */
  void
  run(const Timer::StartClock::duration &startIn = std::chrono::seconds(1));

  void run(const Timer::StartClock::time_point &startAt);

  void run(Int startIn) { run(std::chrono::seconds(startIn)); }
};
} // namespace DPsim
