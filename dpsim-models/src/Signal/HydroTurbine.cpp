// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/Signal/HydroTurbine.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::HydroTurbine::HydroTurbine(const String &name,
                                   CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel), mX1(mAttributes->create<Real>("X1")),
      mPm(mAttributes->create<Real>("Pm")) {}

void HydroTurbine::setParameters(
    std::shared_ptr<Base::TurbineParameters> parameters) {
  auto params =
      std::dynamic_pointer_cast<Signal::HydroTurbineParameters>(parameters);
  if (!params) {
    SPDLOG_LOGGER_ERROR(
        mSLog,
        "Type of parameters class of {} has to be HydroTurbineParameters!",
        this->name());
    throw CPS::TypeException();
  }
  if (params->Tw == 0) {
    SPDLOG_LOGGER_ERROR(mSLog, "Tw must not be zero for {}", this->name());
    throw CPS::InvalidArgumentException();
  }
  mParameters = params;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Hydro turbine parameters:"
                     "\nTw: {:e}",
                     mParameters->Tw);
  mSLog->flush();
}

void HydroTurbine::initializeStates(Real Pminit) {
  if (Pminit < 0 || Pminit > 1) {
    SPDLOG_LOGGER_ERROR(
        mSLog,
        "Initial mechanical power of hydro turbine {} must be in [0, 1] pu",
        this->name());
    throw CPS::InvalidArgumentException();
  }
  **mX1 = Pminit;
  mX1_next = Pminit;
  **mPm = Pminit;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Hydro turbine initial values:"
                     "\nX1: {:f}"
                     "\nPm: {:f}",
                     **mX1, **mPm);
  mSLog->flush();
}

Real HydroTurbine::step(Real Pgv, Real dt) {
  // Shift pre-computed next value into current
  **mX1 = mX1_next;

  // Forward-Euler integration: PT1 with time constant Tw/2
  mX1_next = **mX1 + (2.0 * dt / mParameters->Tw) * (Pgv - **mX1);

  // Mechanical power output
  **mPm = 3.0 * **mX1 - 2.0 * Pgv;

  return **mPm;
}
