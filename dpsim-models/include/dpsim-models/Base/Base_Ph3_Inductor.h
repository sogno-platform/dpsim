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
namespace Ph3 {
class Inductor {
public:
  /// Inductance [H]
  const CPS::Attribute<Matrix>::Ptr mInductance;

  explicit Inductor(CPS::AttributeList::Ptr attributeList)
      : mInductance(attributeList->create<Matrix>("L")){};

  /// Sets model specific parameters
  void setParameters(Matrix inductance) { **mInductance = inductance; }
};
} // namespace Ph3
} // namespace Base
} // namespace CPS
