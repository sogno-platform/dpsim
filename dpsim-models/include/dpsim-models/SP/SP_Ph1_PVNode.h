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

class PVNode : public SimPowerComp<Complex>, public SharedFactory<PVNode> {
public:
  const Attribute<Real>::Ptr mVoltageSetPoint;
  const Attribute<Real>::Ptr mPowerSetPoint;
  const Attribute<Real>::Ptr mVoltagePerUnit;

  PVNode(String uid, String name, Logger::Level logLevel = Logger::Level::off);

  PVNode(String uid, String name, Real power, Real vSetPoint,
         Logger::Level logLevel = Logger::Level::off);

  PVNode(String uid, String name, Real power, Real vSetPoint, Real maxQ,
         Real ratedU, Real ratedS, Logger::Level logLevel = Logger::Level::off);
};

} // namespace Ph1
} // namespace SP
} // namespace CPS
