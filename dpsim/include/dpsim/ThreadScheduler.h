/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/Scheduler.h>

#include <thread>
#include <vector>

namespace DPsim {
class ThreadScheduler : public Scheduler {
public:
  ThreadScheduler(Int threads, String outMeasurementFile,
                  Bool useConditionVariable);
  virtual ~ThreadScheduler();

  void step(Real time, Int timeStepCount);
  virtual void stop();

protected:
  void finishSchedule(const Edges &inEdges);
  void scheduleTask(int thread, CPS::Task::Ptr task);

  Int mNumThreads;

private:
  void doStep(Int scheduleIdx);
  static void threadFunction(ThreadScheduler *sched, Int idx);

  String mOutMeasurementFile;
  Barrier mStartBarrier;

  std::vector<std::thread> mThreads;

  std::vector<CPS::Task::List> mTempSchedules;
  struct ScheduleEntry {
    CPS::Task *task;
    Counter endCounter;
    std::vector<Counter *> reqCounters;
  };
  std::vector<ScheduleEntry *> mSchedules;

  Bool mJoining = false;
  Real mTime = 0;
  Int mTimeStepCount = 0;
};
} // namespace DPsim
