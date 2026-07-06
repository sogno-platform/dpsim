// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph3_TwoTerminalITypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// \brief Generic three-phase, two-terminal I-type SSN component.
///
/// Parametrized directly by arbitrary state-space matrices A, B, C, D
/// (input = port current abc, output = port voltage abc).
class GenericTwoTerminalITypeSSN final
    : public TwoTerminalITypeSSNComp,
      public SharedFactory<GenericTwoTerminalITypeSSN> {
public:
  using SharedFactory<GenericTwoTerminalITypeSSN>::make;

  GenericTwoTerminalITypeSSN(String uid, String name,
                             Logger::Level logLevel = Logger::Level::off);
  GenericTwoTerminalITypeSSN(String name,
                             Logger::Level logLevel = Logger::Level::off)
      : GenericTwoTerminalITypeSSN(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override final;
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
