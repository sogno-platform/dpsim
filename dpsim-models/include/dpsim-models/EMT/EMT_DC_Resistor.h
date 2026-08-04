// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_DC_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace DC {

/// Scalar DC resistor represented as a stateless V-type SSN component.
class Resistor final : public TwoTerminalVTypeSSNComp,
                       public SharedFactory<Resistor> {
public:
  using SharedFactory<Resistor>::make;

  /// Resistance [ohm].
  const Attribute<Real>::Ptr mResistance;

  Resistor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  Resistor(String name, Logger::Level logLevel = Logger::Level::off)
      : Resistor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
  void setParameters(Real resistance);
};

} // namespace DC
} // namespace EMT
} // namespace CPS
