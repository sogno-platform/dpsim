/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/ThreadScheduler.h>

namespace DPsim {
class ThreadListScheduler : public ThreadScheduler {
public:
  ThreadListScheduler(Int threads = 1, String outMeasurementFile = String(),
                      String inMeasurementFile = String(),
                      Bool useConditionVariables = false);

  void createSchedule(const CPS::Task::List &tasks, const Edges &inEdges,
                      const Edges &outEdges);

private:
  String mInMeasurementFile;
};
}; // namespace DPsim
