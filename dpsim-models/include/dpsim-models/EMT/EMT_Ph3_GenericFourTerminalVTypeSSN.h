// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_Ph3_FourTerminalVTypeSSNComp.h>

namespace CPS::EMT::Ph3 {

class GenericFourTerminalVTypeSSN
    : public FourTerminalVTypeSSNComp,
      public SharedFactory<GenericFourTerminalVTypeSSN> {
public:
  using SharedFactory<GenericFourTerminalVTypeSSN>::make;

  GenericFourTerminalVTypeSSN(String uid, String name,
                              Logger::Level logLevel = Logger::Level::off);

  GenericFourTerminalVTypeSSN(String name,
                              Logger::Level logLevel = Logger::Level::off)
      : GenericFourTerminalVTypeSSN(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
};

} // namespace CPS::EMT::Ph3
