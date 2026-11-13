/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include <dpsim-models/Attribute.h>

namespace CPS {
/// \brief Tasks to be defined by every component.
///
/// Each component has to specify its tasks so that the scheduler
/// can use this information. Tasks declare the attributes that are required
/// or modified.
/// The component declares Tasks derived from this general task class.
/// Then, these tasks need to be pushed into the task list of each component
/// so that the scheduler can collect them.
class Task {
public:
  typedef std::shared_ptr<Task> Ptr;
  typedef std::vector<Ptr> List;

  Task() {}
  virtual ~Task() {}

  virtual void execute(Real time, Int timeStepCount) = 0;

  /// Execute through a setup-time compiled function pointer when available.
  /// The virtual execute() API remains the compatibility fallback.
  void executeFast(Real time, Int timeStepCount) {
    if (mFastExecutor)
      mFastExecutor(mFastContext, time, timeStepCount);
    else
      execute(time, timeStepCount);
  }

  template <typename TaskType, void (TaskType::*Executor)(Real, Int)>
  void setFastExecutor(TaskType *task) {
    mFastContext = task;
    mFastExecutor = [](void *context, Real time, Int timeStepCount) {
      (static_cast<TaskType *>(context)->*Executor)(time, timeStepCount);
    };
  }

  virtual String toString() const { return mName; }

  const std::vector<AttributeBase::Ptr> &getAttributeDependencies() {
    return mAttributeDependencies;
  }

  const std::vector<AttributeBase::Ptr> &getModifiedAttributes() {
    return mModifiedAttributes;
  }

  const std::vector<AttributeBase::Ptr> &getPrevStepDependencies() {
    return mPrevStepDependencies;
  }

protected:
  Task(const std::string &name) : mName(name) {}
  std::string mName;
  std::vector<AttributeBase::Ptr> mAttributeDependencies;
  std::vector<AttributeBase::Ptr> mModifiedAttributes;
  std::vector<AttributeBase::Ptr> mPrevStepDependencies;

private:
  using FastExecutor = void (*)(void *, Real, Int);
  FastExecutor mFastExecutor = nullptr;
  void *mFastContext = nullptr;
};
} // namespace CPS
