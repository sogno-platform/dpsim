/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class FIRFilter : public SimSignalComp, public SharedFactory<FIRFilter> {
protected:
  std::vector<Real> mSignal;
  std::vector<Real> mFilter;

  Attribute<Real>::Ptr mInput;
  Int mCurrentIdx;
  Int mFilterLength;

  void incrementIndex();
  Int getIndex(Int index);

public:
  const Attribute<Real>::Ptr mOutput;
  /// This is never explicitely set to reference anything, so the outside code is responsible for setting up the reference.
  const Attribute<Real>::Ptr mInitSample;

  FIRFilter(String uid, String name,
            Logger::Level logLevel = Logger::Level::off);
  FIRFilter(String name, std::vector<Real> filterCoefficients,
            Real initSample = 1, Logger::Level logLevel = Logger::Level::off);

  void initialize(Real timeStep);
  void step(Real time);
  void setInput(Attribute<Real>::Ptr input);
  Task::List getTasks();

  class Step : public Task {
  public:
    Step(FIRFilter &filter) : Task(**filter.mName + ".Step"), mFilter(filter) {
      mAttributeDependencies.push_back(filter.mInput);
      mModifiedAttributes.push_back(filter.mOutput);
    }

    void execute(Real time, Int timeStepCount);

  private:
    FIRFilter &mFilter;
  };
};
} // namespace Signal
} // namespace CPS
