// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Concrete generic three-phase, two-terminal V-type SSN component.
///
/// This component is parametrized directly by arbitrary state-space matrices
/// A, B, C and D.
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

  SimPowerComp<Real>::Ptr clone(String name) override final;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
