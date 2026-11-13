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
#include <fstream>
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

  for (const auto &task : mSchedule)
    SPDLOG_LOGGER_INFO(mSLog, "{}", task->toString());
}

void SequentialScheduler::step(Real time, Int timeStepCount) {
  if (mOutMeasurementFile.size() != 0) {
    const auto stepStart = std::chrono::steady_clock::now();
    std::chrono::nanoseconds taskExecution{};
    for (const auto &task : mSchedule) {
      auto start = std::chrono::steady_clock::now();
      task->executeFast(time, timeStepCount);
      auto end = std::chrono::steady_clock::now();
      const auto elapsed = end - start;
      updateMeasurement(task.get(), elapsed);
      taskExecution +=
          std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
    }
    const auto stepEnd = std::chrono::steady_clock::now();
    mDetailedStepMeasurements.push_back(
        {std::chrono::duration_cast<std::chrono::nanoseconds>(stepEnd -
                                                              stepStart),
         taskExecution, time, timeStepCount});
  } else {
    for (const auto &task : mSchedule) {
      task->executeFast(time, timeStepCount);
    }
  }
}

void SequentialScheduler::stop() {
  if (mOutMeasurementFile.size() != 0) {
    writeMeasurements(mOutMeasurementFile);
    String filename = mOutMeasurementFile;
    const auto extension = filename.rfind(".csv");
    if (extension != String::npos)
      filename.erase(extension);
    filename += "_SchedulerSteps.csv";
    std::ofstream os(filename);
    os << "step,simulation_time_s,scheduler_wall_ns,task_execution_ns,"
          "scheduler_overhead_ns\n";
    for (const auto &measurement : mDetailedStepMeasurements)
      os << measurement.timeStepCount << "," << measurement.simulationTime
         << "," << measurement.schedulerWall.count() << ","
         << measurement.taskExecution.count() << ","
         << measurement.schedulerWall.count() -
                measurement.taskExecution.count()
         << "\n";
  }
}
