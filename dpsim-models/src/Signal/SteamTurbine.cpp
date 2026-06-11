// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/Signal/SteamTurbine.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::SteamTurbine::SteamTurbine(const String &name,
                                   CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mPhp(mAttributes->create<Real>("Php")),
      mPip(mAttributes->create<Real>("Pip")),
      mPlp(mAttributes->create<Real>("Plp")),
      mPm(mAttributes->create<Real>("Pm")) {}

void SteamTurbine::setParameters(
    std::shared_ptr<Base::TurbineParameters> parameters) {
  auto params =
      std::dynamic_pointer_cast<Signal::SteamTurbineParameters>(parameters);
  if (!params) {
    SPDLOG_LOGGER_ERROR(
        mSLog,
        "Type of parameters class of {} has to be SteamTurbineParameters!",
        this->name());
    throw CPS::TypeException();
  }
  Real fractionSum = params->Fhp + params->Fip + params->Flp;
  if (std::abs(fractionSum - 1.0) > 1e-9) {
    SPDLOG_LOGGER_ERROR(
        mSLog,
        "The sum of power fractions Fhp+Fip+Flp of steam turbine {} must equal "
        "1 (got {:f})",
        this->name(), fractionSum);
    throw CPS::InvalidArgumentException();
  }
  mParameters = params;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Steam turbine parameters:"
                     "\nFhp: {:e}"
                     "\nFip: {:e}"
                     "\nFlp: {:e}"
                     "\nTch: {:e}"
                     "\nTrh: {:e}"
                     "\nTco: {:e}",
                     mParameters->Fhp, mParameters->Fip, mParameters->Flp,
                     mParameters->Tch, mParameters->Trh, mParameters->Tco);
  mSLog->flush();
}

void SteamTurbine::initializeStates(Real Pminit) {
  if (Pminit < 0 || Pminit > 1) {
    SPDLOG_LOGGER_ERROR(
        mSLog,
        "Initial mechanical power of steam turbine {} must be in [0, 1] pu",
        this->name());
    throw CPS::InvalidArgumentException();
  }
  **mPhp = Pminit;
  **mPip = Pminit;
  **mPlp = Pminit;
  **mPm = Pminit;
  mPhp_next = Pminit;
  mPip_next = Pminit;
  mPlp_next = Pminit;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Turbine initial values:"
                     "\nPhp: {:f}"
                     "\nPip: {:f}"
                     "\nPlp: {:f}"
                     "\nPm: {:f}",
                     **mPhp, **mPip, **mPlp, **mPm);
  mSLog->flush();
}

Real SteamTurbine::step(Real Pgv, Real dt) {
  // Shift next-step staging into current
  **mPhp = mPhp_next;
  **mPip = mPip_next;
  **mPlp = mPlp_next;

  // High-pressure stage
  if (mParameters->Tch == 0)
    **mPhp = Pgv;
  else
    mPhp_next = **mPhp + (dt / mParameters->Tch) * (Pgv - **mPhp);

  // Intermediate-pressure stage
  if (mParameters->Trh == 0)
    **mPip = **mPhp;
  else
    mPip_next = **mPip + (dt / mParameters->Trh) * (**mPhp - **mPip);

  // Low-pressure stage
  if (mParameters->Tco == 0)
    **mPlp = **mPip;
  else
    mPlp_next = **mPlp + (dt / mParameters->Tco) * (**mPip - **mPlp);

  **mPm = mParameters->Fhp * **mPhp + mParameters->Fip * **mPip +
          mParameters->Flp * **mPlp;

  return **mPm;
}
