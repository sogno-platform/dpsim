/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once

#include <dpsim-models/Base/Base_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
namespace SSN {
/// \brief Inductor
///
/// Three-phase SSN inductor implemented as a specialized two-terminal,
/// V-type SSN component.
///
/// State-space choice:
///   x = i_abc
///   u = v_abc
///   y = i_abc
class Inductor final : public TwoTerminalVTypeSSNComp,
                       public Base::Ph3::Inductor,
                       public SharedFactory<Inductor> {
public:
  using SharedFactory<Inductor>::make;

  /// Defines UID, name, component parameters and logging level
  Inductor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  Inductor(String name, Logger::Level logLevel = Logger::Level::off)
      : Inductor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;

  void setParameters(const Matrix &inductance);
};
} // namespace SSN
} // namespace Ph3
} // namespace EMT
} // namespace CPS
