/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimPowerComp.h>

namespace CPS {
namespace SP {
namespace Ph1 {

class PQNode : public SimPowerComp<Complex>, public SharedFactory<PQNode> {
public:
  const Attribute<Real>::Ptr mPowerNom;
  const Attribute<Real>::Ptr mReactivePowerNom;
  const Attribute<Real>::Ptr mPower;
  const Attribute<Real>::Ptr mReactivePower;

  PQNode(String uid, String name, Logger::Level logLevel = Logger::Level::off);

  PQNode(String uid, String name, Real power, Real reactive_power,
         Logger::Level logLevel = Logger::Level::off);

  void initializeFromTerminal();
  void setPerUnitSystem();
};

} // namespace Ph1
} // namespace SP
} // namespace CPS
