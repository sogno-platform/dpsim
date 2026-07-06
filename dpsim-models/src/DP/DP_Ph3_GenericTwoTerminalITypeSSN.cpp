// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_GenericTwoTerminalITypeSSN.h>

using namespace CPS;

DP::Ph3::GenericTwoTerminalITypeSSN::GenericTwoTerminalITypeSSN(
    String uid, String name, Logger::Level logLevel)
    : TwoTerminalITypeSSNComp(uid, name, logLevel) {}

SimPowerComp<Complex>::Ptr
DP::Ph3::GenericTwoTerminalITypeSSN::clone(String name) {
  auto copy = GenericTwoTerminalITypeSSN::make(name, mLogLevel);
  copy->setParameters(mA, mB, mC, mD);
  return copy;
}
