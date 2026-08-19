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
/// Dynamic Phasor Three-Phase Switch
class Switch {
protected:
  Bool mIsClosedPrev = false;

public:
  /// Resistance if switch is open [ohm]
  const Attribute<Real>::Ptr mOpenResistance;
  /// Resistance if switch is closed [ohm]
  const Attribute<Real>::Ptr mClosedResistance;
  /// Defines if Switch is open or closed
  const Attribute<Bool>::Ptr mIsClosed;

  explicit Switch(CPS::AttributeList::Ptr attributeList)
      : mOpenResistance(attributeList->create<Real>("R_open")),
        mClosedResistance(attributeList->create<Real>("R_closed")),
        mIsClosed(attributeList->create<Bool>("is_closed")){};

  virtual ~Switch() = default;

  ///
  void setParameters(Real openResistance, Real closedResistance,
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
  /// Virtual so current-zero breakers can remain physically conducting after
  /// the command until the current reaches zero.
  virtual void openSwitch() { **mIsClosed = false; }

  /// Close switch
  void close() { closeSwitch(); }
  /// Open switch
  void open() { openSwitch(); }
  /// Check if switch is closed
  Bool isClosed() { return **mIsClosed; }
};
} // namespace Ph1
} // namespace Base
} // namespace CPS
