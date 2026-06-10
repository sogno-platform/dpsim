// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_Ph3_GenericTwoTerminalITypeSSN.h>

using namespace CPS;

EMT::Ph3::GenericTwoTerminalITypeSSN::GenericTwoTerminalITypeSSN(
    String uid, String name, Logger::Level logLevel)
    : TwoTerminalITypeSSNComp(uid, name, logLevel) {}

SimPowerComp<Real>::Ptr
EMT::Ph3::GenericTwoTerminalITypeSSN::clone(String name) {
  auto copy = GenericTwoTerminalITypeSSN::make(name, mLogLevel);
  copy->setParameters(mA, mB, mC, mD);
  return copy;
}
