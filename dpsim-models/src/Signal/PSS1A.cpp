// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/MathUtils.h>
#include <dpsim-models/Signal/PSS1A.h>

using namespace CPS;

Signal::PSS1A::PSS1A(const String &name, CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel) {}

void Signal::PSS1A::setParameters(
    std::shared_ptr<Base::PSSParameters> parameters) {

  if (auto params =
          std::dynamic_pointer_cast<Signal::PSS1AParameters>(parameters)) {
    mParameters = params;

    if (mParameters->Tw == 0) {
      SPDLOG_LOGGER_ERROR(mSLog, "PSS1A: Tw must be non-zero (used as "
                                 "divisor in washout filter)");
      throw CPS::InvalidArgumentException();
    }
    if (mParameters->T2 == 0) {
      SPDLOG_LOGGER_ERROR(mSLog, "PSS1A: T2 must be non-zero (used as "
                                 "divisor in first lead-lag block)");
      throw CPS::InvalidArgumentException();
    }
    if (mParameters->T4 == 0) {
      SPDLOG_LOGGER_ERROR(mSLog, "PSS1A: T4 must be non-zero (used as "
                                 "divisor in second lead-lag block)");
      throw CPS::InvalidArgumentException();
    }

    SPDLOG_LOGGER_INFO(mSLog,
                       "\nPSS1A parameters:"
                       "\nKp: {:e}"
                       "\nKv: {:e}"
                       "\nKw: {:e}"
                       "\nT1: {:e}"
                       "\nT2: {:e}"
                       "\nT3: {:e}"
                       "\nT4: {:e}"
                       "\nMaximum stabilizer output signal: {:e}"
                       "\nMinimum stabilizer output signal: {:e}"
                       "\nTw: {:e}",
                       mParameters->Kp, mParameters->Kv, mParameters->Kw,
                       mParameters->T1, mParameters->T2, mParameters->T3,
                       mParameters->T4, mParameters->Vs_max,
                       mParameters->Vs_min, mParameters->Tw);
    mSLog->flush();

    mA = 1. - mParameters->T1 / mParameters->T2;
    mB = 1. - mParameters->T3 / mParameters->T4;
  } else {
    SPDLOG_LOGGER_ERROR(
        mSLog, "Type of parameters class of {} has to be PSS1AParameters!",
        this->name());
    throw CPS::TypeException();
  }
}

void Signal::PSS1A::initialize(Real omega, Real activePower, Real Vd, Real Vq) {
  Real Vh = sqrt(pow(Vd, 2.) + pow(Vq, 2.));

  mV1 = -(mParameters->Kw * omega + mParameters->Kp * activePower +
          mParameters->Kv * Vh);
  mV2 = mA * (mParameters->Kw * omega + mParameters->Kp * activePower +
              mParameters->Kv * Vh + mV1);
  mV3 =
      mB * (mV2 + (mParameters->T1 / mParameters->T2) *
                      (mParameters->Kw * omega + mParameters->Kp * activePower +
                       mParameters->Kv * Vh + mV1));
  mVs = mV3 + (mParameters->T3 / mParameters->T4) *
                  (mV2 + (mParameters->T1 / mParameters->T2) *
                             (mParameters->Kw * omega +
                              mParameters->Kp * activePower +
                              mParameters->Kv * Vh + mV1));

  mOmega_prev = omega;
  mActivePower_prev = activePower;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\nInitial values:"
                     "\nmV1(t=0): {:e}"
                     "\nmV2(t=0): {:e}"
                     "\nmV3(t=0): {:e}"
                     "\nmVs(t=0): {:e}",
                     mV1, mV2, mV3, mVs);
  mSLog->flush();
}

Real Signal::PSS1A::step(Real omega, Real activePower, Real Vd, Real Vq,
                         Real dt) {
  Real Vh = sqrt(pow(Vd, 2.) + pow(Vq, 2.));
  mVh_prev = Vh;

  mV1_prev = mV1;
  mV2_prev = mV2;
  mV3_prev = mV3;
  mVs_prev = mVs;

  mV1 = mV1_prev - dt / mParameters->Tw *
                       (mParameters->Kw * mOmega_prev +
                        mParameters->Kp * mActivePower_prev +
                        mParameters->Kv * mVh_prev + mV1_prev);
  mV2 = mV2_prev + dt / mParameters->T2 *
                       (mA * (mParameters->Kw * mOmega_prev +
                              mParameters->Kp * mActivePower_prev +
                              mParameters->Kv * mVh_prev + mV1_prev) -
                        mV2_prev);
  mV3 = mV3_prev +
        dt / mParameters->T4 *
            (mB * (mV2_prev + (mParameters->T1 / mParameters->T2) *
                                  (mParameters->Kw * mOmega_prev +
                                   mParameters->Kp * mActivePower_prev +
                                   mParameters->Kv * mVh_prev + mV1_prev)) -
             mV3_prev);

  mVs = mV3 + (mParameters->T3 / mParameters->T4) *
                  (mV2 + (mParameters->T1 / mParameters->T2) *
                             (mParameters->Kw * omega +
                              mParameters->Kp * activePower +
                              mParameters->Kv * Vh + mV1));

  mOmega_prev = omega;
  mActivePower_prev = activePower;
  mVh_prev = Vh;

  if (mVs > mParameters->Vs_max)
    mVs = mParameters->Vs_max;
  else if (mVs < mParameters->Vs_min)
    mVs = mParameters->Vs_min;

  return mVs;
}
