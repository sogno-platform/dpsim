// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph1_TwoTerminalITypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph1 {

/// \brief Generic single-phase, two-terminal I-type SSN component.
///
/// Parametrized directly by arbitrary state-space matrices A, B, C, D
/// (input = port current, output = port voltage).
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

} // namespace Ph1
} // namespace DP
} // namespace CPS
