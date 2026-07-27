// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_DC_Inductor.h>
#include <dpsim-models/EMT/EMT_DC_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace DC {
namespace SSN {

class Inductor final : public TwoTerminalVTypeSSNComp,
                       public Base::DC::Inductor,
                       public SharedFactory<Inductor> {
public:
  using SharedFactory<Inductor>::make;

  Inductor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  Inductor(String name, Logger::Level logLevel = Logger::Level::off)
      : Inductor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
  void setParameters(Real inductance, Real initialCurrent = 0.0);
  void initializeFromNodesAndTerminals(Real frequency) override final;
};

} // namespace SSN
} // namespace DC
} // namespace EMT
} // namespace CPS
