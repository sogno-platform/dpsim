/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/MathUtils.h>
#include <dpsim-models/Signal/TurbineGovernorType1.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::TurbineGovernorType1::TurbineGovernorType1(const String &name,
                                                   CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mXg1(mAttributes->create<Real>("Xg1")),
      mXg2(mAttributes->create<Real>("Xg2")),
      mXg3(mAttributes->create<Real>("Xg3")),
      mTm(mAttributes->create<Real>("Tm")) {}

void TurbineGovernorType1::setParameters(Real T3, Real T4, Real T5, Real Tc,
                                         Real Ts, Real R, Real Tmin, Real Tmax,
                                         Real OmRef) {
  mT3 = T3;
  mT4 = T4;
  mT5 = T5;
  mTc = Tc;
  mTs = Ts;
  mR = R;
  mTmin = Tmin;
  mTmax = Tmax;
  mOmRef = OmRef;

  SPDLOG_LOGGER_INFO(mSLog,
                     "TurbineGovernorType1 parameters: "
                     "\nT3: {:e}"
                     "\nT4: {:e}"
                     "\nT5: {:e}"
                     "\nTc: {:e}"
                     "\nTs: {:e}"
                     "\nR: {:e}"
                     "\nTmin: {:e}"
                     "\nTmax: {:e}"
                     "\nOmRef: {:e}",
                     mT3, mT4, mT5, mTc, mTs, mR, mTmin, mTmax, mOmRef);
}

void TurbineGovernorType1::initialize(Real TmRef) {
  mTmRef = TmRef;
  **mXg1 = TmRef;
  **mXg2 = (1 - mT3 / mTc) * **mXg1;
  **mXg3 = (1 - mT4 / mT5) * (**mXg2 + mT3 / mTc * **mXg1);
  **mTm = **mXg3 + mT4 / mT5 * (**mXg2 + mT3 / mTc * **mXg1);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Governor initial values: \n"
                     "\nTorder: {:f}"
                     "\nXg1: {:f}"
                     "\nXg2: {:f}"
                     "\nXg3: {:f}"
                     "\nTm: {:f}",
                     mTmRef, **mXg1, **mXg2, **mXg3, **mTm);
}

Real TurbineGovernorType1::step(Real Omega, Real dt) {

  /// update state variables at time k-1
  mXg1_prev = **mXg1;
  mXg2_prev = **mXg2;
  mXg3_prev = **mXg3;

  /// Input of speed relay
  Real Tin = mTmRef + (mOmRef - Omega) / mR;
  if (Tin > mTmax)
    Tin = mTmax;
  if (Tin < mTmin)
    Tin = mTmin;

  /// Governor
  **mXg1 = Math::StateSpaceEuler(mXg1_prev, -1 / mTs, 1 / mTs, dt, Tin);

  /// Servo
  **mXg2 = Math::StateSpaceEuler(mXg2_prev, -1 / mTc, (1 - mT3 / mTc) / mTc, dt,
                                 mXg1_prev);

  /// Reheat
  **mXg3 = Math::StateSpaceEuler(mXg3_prev, -1 / mT5, (1 - mT4 / mT5) / mT5, dt,
                                 (mXg2_prev + mT3 / mTc * mXg1_prev));

  /// Mechanical torque
  **mTm = **mXg3 + mT4 / mT5 * (**mXg2 + mT3 / mTc * **mXg1);

  return **mTm;
}
