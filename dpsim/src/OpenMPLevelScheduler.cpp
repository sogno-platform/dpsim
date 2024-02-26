/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/OpenMPLevelScheduler.h>
#include <omp.h>

#include <iostream>

using namespace CPS;
using namespace DPsim;

OpenMPLevelScheduler::OpenMPLevelScheduler(Int threads,
                                           String outMeasurementFile)
    : mOutMeasurementFile(outMeasurementFile) {
  if (threads >= 0)
    mNumThreads = threads;
  else
    mNumThreads = omp_get_num_threads();
}

void OpenMPLevelScheduler::createSchedule(const Task::List &tasks,
                                          const Edges &inEdges,
                                          const Edges &outEdges) {
  Task::List ordered;

  Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
  Scheduler::levelSchedule(ordered, inEdges, outEdges, mLevels);

  if (!mOutMeasurementFile.empty())
    Scheduler::initMeasurements(tasks);
}

void OpenMPLevelScheduler::step(Real time, Int timeStepCount) {
  long i, level = 0;
  std::chrono::steady_clock::time_point start, end;

  if (!mOutMeasurementFile.empty()) {
#pragma omp parallel shared(time, timeStepCount) private(level, i, start, end) \
    num_threads(mNumThreads)
    for (level = 0; level < static_cast<long>(mLevels.size()); level++) {
      {
#pragma omp for schedule(static)
        for (i = 0; i < static_cast<long>(mLevels[level].size()); i++) {
          start = std::chrono::steady_clock::now();
          mLevels[level][i]->execute(time, timeStepCount);
          end = std::chrono::steady_clock::now();
          updateMeasurement(mLevels[level][i].get(), end - start);
        }
      }
    }
  } else {
#pragma omp parallel shared(time, timeStepCount) private(level, i)             \
    num_threads(mNumThreads)
    for (level = 0; level < static_cast<long>(mLevels.size()); level++) {
      {
#pragma omp for schedule(static)
        for (i = 0; i < static_cast<long>(mLevels[level].size()); i++) {
          mLevels[level][i]->execute(time, timeStepCount);
        }
      }
    }
  }
}

void OpenMPLevelScheduler::stop() {
  if (!mOutMeasurementFile.empty()) {
    writeMeasurements(mOutMeasurementFile);
  }
}
