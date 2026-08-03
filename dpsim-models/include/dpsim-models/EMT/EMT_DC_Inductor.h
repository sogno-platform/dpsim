// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_DC_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace DC {

/// Scalar DC inductor represented as a one-state V-type SSN component.
class Inductor final : public TwoTerminalVTypeSSNComp,
                       public SharedFactory<Inductor> {
public:
  using SharedFactory<Inductor>::make;

  /// Inductance [H].
  const Attribute<Real>::Ptr mInductance;
  /// Initial current from terminal 1 to terminal 0 [A].
  const Attribute<Real>::Ptr mInitialCurrent;

  Inductor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  Inductor(String name, Logger::Level logLevel = Logger::Level::off)
      : Inductor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
  void setParameters(Real inductance, Real initialCurrent = 0.0);
  void initializeFromNodesAndTerminals(Real frequency) override final;
};

} // namespace DC
} // namespace EMT
} // namespace CPS
