// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/Signal/SteamTurbineGovernor.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::SteamTurbineGovernor::SteamTurbineGovernor(const String &name,
                                                   CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mDelOm(mAttributes->create<Real>("DelOm")),
      mP1(mAttributes->create<Real>("P1")), mP(mAttributes->create<Real>("P")),
      mPgvLim(mAttributes->create<Real>("PgvLim")),
      mPgv(mAttributes->create<Real>("Pgv")) {}

void SteamTurbineGovernor::setParameters(
    std::shared_ptr<Base::GovernorParameters> parameters) {
  auto params =
      std::dynamic_pointer_cast<Signal::SteamGorvernorParameters>(parameters);
  if (!params) {
    SPDLOG_LOGGER_ERROR(
        mSLog,
        "Type of parameters class of {} has to be SteamGorvernorParameters!",
        this->name());
    throw CPS::TypeException();
  }
  if (params->T3 == 0) {
    SPDLOG_LOGGER_ERROR(mSLog, "T3 must not be zero for {}", this->name());
    throw CPS::InvalidArgumentException();
  }
  if (params->R == 0) {
    SPDLOG_LOGGER_ERROR(mSLog, "R must not be zero for {}", this->name());
    throw CPS::InvalidArgumentException();
  }
  mParameters = params;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nSteam Governor parameters:"
                     "\nOmRef: {:e}"
                     "\nR: {:e}"
                     "\nT1: {:e}"
                     "\nT2: {:e}"
                     "\nT3: {:e}"
                     "\ndPmax: {:e}"
                     "\ndPmin: {:e}"
                     "\nPmax: {:e}"
                     "\nPmin: {:e}"
                     "\nKbc: {:e}\n",
                     mParameters->OmRef, mParameters->R, mParameters->T1,
                     mParameters->T2, mParameters->T3, mParameters->dPmax,
                     mParameters->dPmin, mParameters->Pmax, mParameters->Pmin,
                     mParameters->Kbc);
  mSLog->flush();
}

void SteamTurbineGovernor::initializeStates(Real Pref) {
  if (Pref < 0 || Pref > 1) {
    SPDLOG_LOGGER_ERROR(mSLog, "Pref of steam governor {} must be in [0, 1] pu",
                        this->name());
    throw CPS::InvalidArgumentException();
  }
  mPref = Pref;
  **mDelOm = 0;
  mDelOm_prev = 0;
  mDelOm_2prev = 0;
  **mP1 = 0;
  mP1_prev = 0;
  **mP = 0;
  mDerPgv = 0;
  **mPgvLim = Pref;
  **mPgv = Pref;

  if (mParameters->T1 == 0) {
    mCa = 0;
    mCb = 0;
  } else {
    mCa = mParameters->T2 / mParameters->T1;
    mCb = (mParameters->T1 - mParameters->T2) / mParameters->T1;
  }

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nSteam Governor initial values:"
                     "\nPref: {:f}"
                     "\nDelOm: {:f}"
                     "\nDerPgv: {:f}"
                     "\nPgv: {:f}",
                     mPref, **mDelOm, mDerPgv, **mPgv);
  mSLog->flush();
}

Real SteamTurbineGovernor::step(Real Omega, Real dt) {
  // Shift state variables one step back
  mDelOm_2prev = mDelOm_prev;
  mDelOm_prev = **mDelOm;
  mP1_prev = **mP1;

  // Compute input deviation
  **mDelOm = mParameters->OmRef - Omega;

  // Lead-lag controller K(1+sT2)/(1+sT1): if T1==0, use filtered-derivative form
  if (mParameters->T1 == 0) {
    **mP =
        (1.0 / mParameters->R) *
        (mDelOm_prev + (mParameters->T2 / dt) * (mDelOm_prev - mDelOm_2prev));
  } else {
    **mP1 = mP1_prev + (dt / mParameters->T1) * (mDelOm_prev * mCb - mP1_prev);
    **mP = (1.0 / mParameters->R) * (mP1_prev + mDelOm_prev * mCa);
  }

  // Integrator with rate limiter and anti-windup
  mDerPgv = (1.0 / mParameters->T3) * (mPref + **mP - **mPgv);
  if (mDerPgv < mParameters->dPmin)
    mDerPgv = mParameters->dPmin;
  if (mDerPgv > mParameters->dPmax)
    mDerPgv = mParameters->dPmax;
  mDerPgv = mDerPgv - mParameters->Kbc * (**mPgvLim - **mPgv);

  **mPgvLim = **mPgvLim + dt * mDerPgv;

  // Output limiter
  if (**mPgvLim < mParameters->Pmin)
    **mPgv = mParameters->Pmin;
  else if (**mPgvLim > mParameters->Pmax)
    **mPgv = mParameters->Pmax;
  else
    **mPgv = **mPgvLim;

  return **mPgv;
}
