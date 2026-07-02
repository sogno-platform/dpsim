// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph3_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// \brief Generic three-phase, two-terminal V-type SSN component.
///
/// Parametrized directly by arbitrary state-space matrices A, B, C, D
/// (input = port voltage abc, output = port current abc).
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

} // namespace Ph3
} // namespace DP
} // namespace CPS
