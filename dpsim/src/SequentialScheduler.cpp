/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/SequentialScheduler.h>

using namespace CPS;
using namespace DPsim;

#include <chrono>
#include <deque>
#include <iostream>
#include <set>
#include <typeinfo>
#include <unordered_map>

void SequentialScheduler::createSchedule(const Task::List &tasks,
                                         const Edges &inEdges,
                                         const Edges &outEdges) {
  if (mOutMeasurementFile.size() != 0)
    Scheduler::initMeasurements(tasks);
  Scheduler::topologicalSort(tasks, inEdges, outEdges, mSchedule);

  for (auto task : mSchedule)
    SPDLOG_LOGGER_INFO(mSLog, "{}", task->toString());
}

void SequentialScheduler::step(Real time, Int timeStepCount) {
  if (mOutMeasurementFile.size() != 0) {
    for (auto task : mSchedule) {
      auto start = std::chrono::steady_clock::now();
      task->execute(time, timeStepCount);
      auto end = std::chrono::steady_clock::now();
      updateMeasurement(task.get(), end - start);
    }
  } else {
    for (auto it : mSchedule) {
      it->execute(time, timeStepCount);
    }
  }
}

void SequentialScheduler::stop() {
  if (mOutMeasurementFile.size() != 0)
    writeMeasurements(mOutMeasurementFile);
}
