/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {
namespace Ph1 {
class Resistor {
public:
  ///Resistance [ohm]
  const CPS::Attribute<Real>::Ptr mResistance;

  explicit Resistor(CPS::AttributeList::Ptr attributeList)
      : mResistance(attributeList->create<Real>("R")){};

  ///
  void setParameters(Real resistance) { **mResistance = resistance; }
};
} // namespace Ph1
} // namespace Base
} // namespace CPS
