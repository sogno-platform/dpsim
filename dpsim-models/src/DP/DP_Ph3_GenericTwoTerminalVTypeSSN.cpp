// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_GenericTwoTerminalVTypeSSN.h>

using namespace CPS;

DP::Ph3::GenericTwoTerminalVTypeSSN::GenericTwoTerminalVTypeSSN(
    String uid, String name, Logger::Level logLevel)
    : TwoTerminalVTypeSSNComp(uid, name, logLevel) {}

SimPowerComp<Complex>::Ptr
DP::Ph3::GenericTwoTerminalVTypeSSN::clone(String name) {
  auto copy = GenericTwoTerminalVTypeSSN::make(name, mLogLevel);
  copy->setParameters(mA, mB, mC, mD);
  return copy;
}
