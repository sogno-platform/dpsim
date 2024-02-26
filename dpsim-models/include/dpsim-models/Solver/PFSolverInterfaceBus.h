/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MathUtils.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim-models/SP/SP_Ph1_PQNode.h>
#include <dpsim-models/SP/SP_Ph1_PVNode.h>
#include <dpsim-models/SP/SP_Ph1_VDNode.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/SimTerminal.h>
#include <dpsim-models/Task.h>

namespace CPS {
/// Common base class of all Component templates.
class PFSolverInterfaceBus {
protected:
  Task::List mPFTasks;

public:
  typedef std::shared_ptr<PFSolverInterfaceBus> Ptr;
  typedef std::vector<Ptr> List;

  std::shared_ptr<CPS::SP::Ph1::PQNode> mPQ;
  std::shared_ptr<CPS::SP::Ph1::PVNode> mPV;
  std::shared_ptr<CPS::SP::Ph1::VDNode> mVD;

  /// Define the type of bus the component is modelled by
  PowerflowBusType mPowerflowBusType;
  PFSolverInterfaceBus() = default;

  virtual void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) = 0;
  virtual void pfBusInitialize() { mPFTasks.clear(); }
  const Task::List &pfTasks() { return mPFTasks; }
};

} // namespace CPS
