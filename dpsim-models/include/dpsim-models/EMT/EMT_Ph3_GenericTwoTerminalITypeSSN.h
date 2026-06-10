// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalITypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Concrete generic three-phase, two-terminal I-type SSN component.
///
/// This component is parametrized directly by arbitrary state-space matrices
/// A, B, C and D.
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

  SimPowerComp<Real>::Ptr clone(String name) override final;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
