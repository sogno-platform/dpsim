/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/ThreadLevelScheduler.h>

#include <algorithm>
#include <iostream>
#include <typeinfo>

using namespace CPS;
using namespace DPsim;

ThreadLevelScheduler::ThreadLevelScheduler(Int threads,
                                           String outMeasurementFile,
                                           String inMeasurementFile,
                                           Bool useConditionVariable,
                                           Bool sortTaskTypes)
    : ThreadScheduler(threads, outMeasurementFile, useConditionVariable),
      mInMeasurementFile(inMeasurementFile), mSortTaskTypes(sortTaskTypes) {}

void ThreadLevelScheduler::createSchedule(const Task::List &tasks,
                                          const Edges &inEdges,
                                          const Edges &outEdges) {
  Task::List ordered;
  std::vector<Task::List> levels;

  Scheduler::topologicalSort(tasks, inEdges, outEdges, ordered);
  Scheduler::initMeasurements(ordered);

  Scheduler::levelSchedule(ordered, inEdges, outEdges, levels);

  if (!mInMeasurementFile.empty()) {
    std::unordered_map<String, TaskTime::rep> measurements;
    readMeasurements(mInMeasurementFile, measurements);
    for (size_t level = 0; level < levels.size(); level++) {
      // Distribute tasks such that the execution time is (approximately) minimized
      scheduleLevel(levels[level], measurements, inEdges);
    }
  } else {
    for (size_t level = 0; level < levels.size(); level++) {
      if (mSortTaskTypes)
        sortTasksByType(levels[level].begin(), levels[level].end());
      // Distribute tasks of one level evenly between threads
      for (Int thread = 0; thread < mNumThreads; ++thread) {
        Int start =
            static_cast<Int>(levels[level].size()) * thread / mNumThreads;
        Int end =
            static_cast<Int>(levels[level].size()) * (thread + 1) / mNumThreads;
        for (int idx = start; idx != end; idx++)
          scheduleTask(thread, levels[level][idx]);
      }
    }
  }

  ThreadScheduler::finishSchedule(inEdges);
}

void ThreadLevelScheduler::sortTasksByType(Task::List::iterator begin,
                                           CPS::Task::List::iterator end) {
  auto cmp = [](const Task::Ptr &p1, const Task::Ptr &p2) -> bool {
  // TODO: according to the standard, the ordering may change between invocations
  // clang complains here for some reason that the expressions in the typeid
  // might be evaluated (which is the whole point)
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpotentially-evaluated-expression"
#endif
    return typeid(*p1).before(typeid(*p2));
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  };
  std::sort(begin, end, cmp);
}

void ThreadLevelScheduler::scheduleLevel(
    const Task::List &tasks,
    const std::unordered_map<String, TaskTime::rep> &measurements,
    const Edges &inEdges) {
  Task::List tasksSorted = tasks;

  // Check that measurements map is complete
  for (auto task : tasks) {
    if (measurements.find(task->toString()) == measurements.end())
      throw SchedulingException();
  }

  if (mSortTaskTypes) {
    TaskTime::rep totalTime = 0;
    for (auto task : tasks) {
      totalTime += measurements.at(task->toString());
    }

    TaskTime::rep avgTime = totalTime / mNumThreads;

    // Sort the tasks by type and then push them to the threads in order
    // while aiming for approximately equal execution time.
    // Should work well enough for a large enough number of tasks
    sortTasksByType(tasksSorted.begin(), tasksSorted.end());
    size_t task = 0;
    for (int thread = 0; thread < mNumThreads; thread++) {
      TaskTime::rep curTime = 0;
      while (curTime < avgTime && task < tasksSorted.size()) {
        scheduleTask(thread, tasksSorted[task]);
        curTime += measurements.at(tasksSorted[task]->toString());
        task++;
      }
    }
    // All tasks should be distributed, but just to be sure, put the remaining
    // ones to the last thread
    for (; task < tasksSorted.size(); task++)
      scheduleTask(mNumThreads - 1, tasksSorted[task]);
  } else {
    // Sort tasks in descending execution time
    auto cmp = [&measurements](const Task::Ptr &p1,
                               const Task::Ptr &p2) -> bool {
      return measurements.at(p1->toString()) > measurements.at(p2->toString());
    };
    std::sort(tasksSorted.begin(), tasksSorted.end(), cmp);

    // Greedy heuristic: schedule the tasks to the thread with the smallest current execution time
    std::vector<TaskTime::rep> totalTimes(mNumThreads, 0);
    for (auto task : tasksSorted) {
      auto minIt = std::min_element(totalTimes.begin(), totalTimes.end());
      Int minIdx = static_cast<UInt>(minIt - totalTimes.begin());
      scheduleTask(minIdx, task);
      totalTimes[minIdx] += measurements.at(task->toString());
    }
  }
}
