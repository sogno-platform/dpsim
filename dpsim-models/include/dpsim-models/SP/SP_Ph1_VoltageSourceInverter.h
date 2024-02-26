/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include "dpsim-models/Solver/PFSolverInterfaceBus.h"
#include <dpsim-models/SP/SP_Ph1_PQNode.h>
#include <dpsim-models/SP/SP_Ph1_PVNode.h>
#include <dpsim-models/SP/SP_Ph1_VDNode.h>
#include <dpsim-models/SimPowerComp.h>

namespace CPS {

namespace SP {
namespace Ph1 {
class VoltageSourceInverter : public SimPowerComp<Complex>,
                              public SharedFactory<VoltageSourceInverter>,
                              public PFSolverInterfaceBus {
public:
  VoltageSourceInverter(
      String uid, String name, Real power, Real reactivePower,
      PowerflowBusType powerflowBusType = PowerflowBusType::PQ,
      Logger::Level logLevel = Logger::Level::off);

  VoltageSourceInverter(
      String uid, String name,
      PowerflowBusType powerflowBusType = PowerflowBusType::PQ,
      Logger::Level logLevel = Logger::Level::off);

  // #### Powerflow section ####
  /// Modify powerflow bus type
  void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
