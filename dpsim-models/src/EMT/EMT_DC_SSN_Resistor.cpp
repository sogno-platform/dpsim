// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_SSN_Resistor.h>

using namespace CPS;

EMT::DC::SSN::Resistor::Resistor(String uid, String name,
                                 Logger::Level logLevel)
    : TwoTerminalVTypeSSNComp(uid, name, logLevel),
      Base::DC::Resistor(mAttributes) {}

SimPowerComp<Real>::Ptr EMT::DC::SSN::Resistor::clone(String name) {
  auto copy = Resistor::make(name, mLogLevel);
  copy->setParameters(**mResistance);
  return copy;
}

void EMT::DC::SSN::Resistor::setParameters(Real resistance) {
  if (!Math::isFinite(resistance) ||
      resistance <= std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC resistance must be finite and safely greater than zero.");

  **mResistance = resistance;
  SSNComp::setParameters(Matrix::Zero(0, 0), Matrix::Zero(0, 1),
                         Matrix::Zero(1, 0),
                         Matrix::Constant(1, 1, 1.0 / resistance));
}
