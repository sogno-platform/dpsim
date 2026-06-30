// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph1_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph1 {

/// \brief Generic single-phase, two-terminal V-type SSN component.
///
/// Parametrized directly by arbitrary state-space matrices A, B, C, D
/// (input = port voltage, output = port current).
class GenericTwoTerminalVTypeSSN final
    : public TwoTerminalVTypeSSNComp,
      public SharedFactory<GenericTwoTerminalVTypeSSN> {
public:
  using SharedFactory<GenericTwoTerminalVTypeSSN>::make;

  GenericTwoTerminalVTypeSSN(String uid, String name,
                             Logger::Level logLevel = Logger::Level::off);
  GenericTwoTerminalVTypeSSN(String name,
                             Logger::Level logLevel = Logger::Level::off)
      : GenericTwoTerminalVTypeSSN(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override final;
};

} // namespace Ph1
} // namespace DP
} // namespace CPS
