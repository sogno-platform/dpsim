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
};
} // namespace CPS
