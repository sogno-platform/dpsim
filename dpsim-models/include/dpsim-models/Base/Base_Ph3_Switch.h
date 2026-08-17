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

/// Common base for three-phase switches.
class Switch {
protected:
  Bool mIsClosedPrev = false;

public:
  /// Resistance if switch is open [ohm]
  const CPS::Attribute<Matrix>::Ptr mOpenResistance;
  /// Resistance if switch is closed [ohm]
  const CPS::Attribute<Matrix>::Ptr mClosedResistance;
  /// Commanded/global switch state.
  const CPS::Attribute<Bool>::Ptr mIsClosed;

  explicit Switch(CPS::AttributeList::Ptr attributeList)
      : mOpenResistance(attributeList->create<Matrix>("R_open")),
        mClosedResistance(attributeList->create<Matrix>("R_closed")),
        mIsClosed(attributeList->create<Bool>("is_closed")) {}

  virtual ~Switch() = default;

  void setParameters(Matrix openResistance, Matrix closedResistance,
                     Bool closed = false) {
    **mOpenResistance = openResistance;
    **mClosedResistance = closedResistance;
    **mIsClosed = closed;
  }

  /// Close command.
  ///
  /// Virtual so domain-specific breaker models can distinguish commanded
  /// state from the physical pole state.
  virtual void closeSwitch() { **mIsClosed = true; }

  /// Open command.
  ///
  /// Virtual so EMT/DP current-zero breakers can remain physically conducting
  /// after the command until the individual phase currents reach zero.
  virtual void openSwitch() { **mIsClosed = false; }

  /// Check commanded/global state.
  Bool isClosed() { return **mIsClosed; }
};

} // namespace Ph3
} // namespace Base
} // namespace CPS
