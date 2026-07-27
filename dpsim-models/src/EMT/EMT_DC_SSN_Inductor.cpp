// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/EMT/EMT_DC_SSN_Inductor.h>

using namespace CPS;

EMT::DC::SSN::Inductor::Inductor(String uid, String name,
                                 Logger::Level logLevel)
    : TwoTerminalVTypeSSNComp(uid, name, logLevel),
      Base::DC::Inductor(mAttributes) {}

SimPowerComp<Real>::Ptr EMT::DC::SSN::Inductor::clone(String name) {
  auto copy = Inductor::make(name, mLogLevel);
  copy->setParameters(**mInductance, **mInitialCurrent);
  return copy;
}

void EMT::DC::SSN::Inductor::setParameters(Real inductance,
                                           Real initialCurrent) {
  if (!Math::isFinite(inductance) ||
      inductance <= std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC inductance must be finite and safely greater than zero.");
  if (!Math::isFinite(initialCurrent))
    throw std::invalid_argument("Initial inductor current must be finite.");

  **mInductance = inductance;
  **mInitialCurrent = initialCurrent;

  // x = iL, u = v, y = iL
  SSNComp::setParameters(Matrix::Zero(1, 1),
                         Matrix::Constant(1, 1, 1.0 / inductance),
                         Matrix::Identity(1, 1), Matrix::Zero(1, 1));
}

void EMT::DC::SSN::Inductor::initializeFromNodesAndTerminals(Real) {
  if (!mParametersSet)
    throw std::logic_error(
        "setParameters() must be called before inductor initialization.");
  validateDCTerminals();

  const Complex voltage = initialSingleVoltage(1) - initialSingleVoltage(0);
  if (!Math::isFinite(voltage) ||
      std::abs(voltage.imag()) > std::numeric_limits<Real>::epsilon())
    throw std::invalid_argument(
        "DC initial inductor voltage must be a finite real scalar value.");

  (**mX)(0, 0) = **mInitialCurrent;
  (**mIntfVoltage)(0, 0) = voltage.real();
  (**mIntfCurrent)(0, 0) = **mInitialCurrent;
}
