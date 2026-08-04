// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_DC_TwoTerminalITypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace DC {

/// Scalar DC capacitor represented as a one-state I-type SSN component.
class Capacitor final : public TwoTerminalITypeSSNComp,
                        public SharedFactory<Capacitor> {
public:
  using SharedFactory<Capacitor>::make;

  /// Capacitance [F].
  const Attribute<Real>::Ptr mCapacitance;

  Capacitor(String uid, String name,
            Logger::Level logLevel = Logger::Level::off);
  Capacitor(String name, Logger::Level logLevel = Logger::Level::off)
      : Capacitor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
  void setParameters(Real capacitance);
  void initializeFromNodesAndTerminals(Real frequency) override final;
};

} // namespace DC
} // namespace EMT
} // namespace CPS
