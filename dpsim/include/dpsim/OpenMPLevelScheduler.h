/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/Scheduler.h>

#include <vector>

namespace DPsim {
class OpenMPLevelScheduler : public Scheduler {
public:
  OpenMPLevelScheduler(Int threads = -1, String outMeasurementFile = String());
  void createSchedule(const CPS::Task::List &tasks, const Edges &inEdges,
                      const Edges &outEdges);
  void step(Real time, Int timeStepCount);
  void stop();

private:
  Int mNumThreads;
  String mOutMeasurementFile;
  std::vector<CPS::Task::List> mLevels;
};
}; // namespace DPsim
