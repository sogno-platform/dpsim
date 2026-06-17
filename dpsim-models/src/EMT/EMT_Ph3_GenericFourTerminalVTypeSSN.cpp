// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_GenericFourTerminalVTypeSSN.h>

using namespace CPS;

EMT::Ph3::GenericFourTerminalVTypeSSN::GenericFourTerminalVTypeSSN(
    String uid, String name, Logger::Level logLevel)
    : FourTerminalVTypeSSNComp(uid, name, logLevel) {}

SimPowerComp<Real>::Ptr
EMT::Ph3::GenericFourTerminalVTypeSSN::clone(String name) {
  auto copy = GenericFourTerminalVTypeSSN::make(name, mLogLevel);
  copy->setParameters(mA, mB, mC, mD);
  return copy;
}
