// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_SSN_Capacitor.h>

using namespace CPS;

EMT::DC::SSN::Capacitor::Capacitor(String uid, String name,
                                   Logger::Level logLevel)
    : TwoTerminalITypeSSNComp(uid, name, logLevel),
      Base::DC::Capacitor(mAttributes) {}

SimPowerComp<Real>::Ptr EMT::DC::SSN::Capacitor::clone(String name) {
  auto copy = Capacitor::make(name, mLogLevel);
  copy->setParameters(**mCapacitance);
  return copy;
}

void EMT::DC::SSN::Capacitor::setParameters(Real capacitance) {
  if (!Math::isFinite(capacitance) ||
      capacitance <= std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC capacitance must be finite and safely greater than zero.");

  **mCapacitance = capacitance;

  // x = vC, u = i, y = vC
  SSNComp::setParameters(Matrix::Zero(1, 1),
                         Matrix::Constant(1, 1, 1.0 / capacitance),
                         Matrix::Identity(1, 1), Matrix::Zero(1, 1));
}

void EMT::DC::SSN::Capacitor::initializeFromNodesAndTerminals(Real) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before capacitor initialization.");
  validateDCTerminals();

  const Complex voltage = initialSingleVoltage(1) - initialSingleVoltage(0);
  if (!Math::isFinite(voltage) ||
      std::abs(voltage.imag()) > std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC initial capacitor voltage must be a finite real scalar value.");

  (**mX)(0, 0) = voltage.real();
  (**mIntfVoltage)(0, 0) = voltage.real();
  (**mIntfCurrent)(0, 0) = 0.0;
}
