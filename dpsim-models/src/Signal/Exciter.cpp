/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/MathUtils.h>
#include <dpsim-models/Signal/Exciter.h>

using namespace CPS;

Signal::Exciter::Exciter(const String &name, CPS::Logger::Level logLevel)
    : SimSignalComp(name, name, logLevel),
      mVm(mAttributes->create<Real>("Vm", 0)),
      mVh(mAttributes->create<Real>("Vh", 0)),
      mVis(mAttributes->create<Real>("Vis", 0)),
      mVse(mAttributes->create<Real>("Vse", 0)),
      mVr(mAttributes->create<Real>("Vr", 0)),
      mEf(mAttributes->create<Real>("Ef", 0)) {}

void Signal::Exciter::setParameters(Real Ta, Real Ka, Real Te, Real Ke, Real Tf,
                                    Real Kf, Real Tr, Real maxVr, Real minVr) {
  mTa = Ta;
  mKa = Ka;
  mTe = Te;
  mKe = Ke;
  mTf = Tf;
  mKf = Kf;
  mTr = Tr;
  mMaxVr = maxVr;
  mMinVr = minVr;

  SPDLOG_LOGGER_INFO(mSLog,
                     "Exciter parameters: \n"
                     "Ta: {:e}"
                     "\nKa: {:e}"
                     "\nTe: {:e}"
                     "\nKe: {:e}"
                     "\nTf: {:e}"
                     "\nKf: {:e}"
                     "\nTr: {:e}"
                     "\nMaximum regulator Voltage: {:e}"
                     "\nMinimum regulator Voltage: {:e}\n",
                     mTa, mKa, mTe, mKe, mTf, mKf, mTr, mMaxVr, mMinVr);
}

void Signal::Exciter::initialize(Real Vh_init, Real Ef_init) {

  SPDLOG_LOGGER_INFO(mSLog,
                     "Initially set excitation system initial values: \n"
                     "Vh_init: {:e}\nEf_init: {:e}\n",
                     Vh_init, Ef_init);

  **mVm = Vh_init;
  **mEf = Ef_init;

  // mVse is the ceiling function in PSAT
  // mVse = mEf * (0.33 * (exp(0.1 * abs(mEf)) - 1.));
  **mVse = **mEf * (0.33 * exp(0.1 * abs(**mEf)));

  // mVis = vr2 in PSAT
  **mVis = -mKf / mTf * **mEf;

  // mVr = vr1 in PSAT
  **mVr = mKe * **mEf + **mVse;
  if (**mVr > mMaxVr)
    **mVr = mMaxVr;
  else if (**mVr < mMinVr)
    **mVr = mMinVr;

  mVref = **mVr / mKa + **mVm;
  SPDLOG_LOGGER_INFO(mSLog,
                     "Actually applied excitation system initial values:"
                     "\nVref : {:e}"
                     "\ninit_Vm: {:e}"
                     "\ninit_Ef: {:e}"
                     "\ninit_Vc: {:e}"
                     "\ninit_Vr: {:e}"
                     "\ninit_Vr2: {:e}",
                     mVref, **mVm, **mEf, **mVse, **mVr, **mVis);
}

Real Signal::Exciter::step(Real mVd, Real mVq, Real dt) {
  // Voltage magnitude calculation
  **mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));

  // update state variables at time k-1
  mVm_prev = **mVm;
  mVis_prev = **mVis;
  mVr_prev = **mVr;
  mEf_prev = **mEf;

  // compute state variables at time k using euler forward

  // Voltage Transducer equation
  **mVm = Math::StateSpaceEuler(mVm_prev, -1 / mTr, 1 / mTr, dt, **mVh);

  // Stabilizing feedback equation
  // mVse = mEf * (0.33 * (exp(0.1 * abs(mEf)) - 1.));
  **mVse = mEf_prev * (0.33 * exp(0.1 * abs(mEf_prev)));
  **mVis = Math::StateSpaceEuler(mVis_prev, -1 / mTf, -mKf / mTf / mTf, dt,
                                 mEf_prev);

  // Voltage regulator equation
  **mVr =
      Math::StateSpaceEuler(mVr_prev, -1 / mTa, mKa / mTa, dt,
                            mVref - **mVm - mVis_prev - mKf / mTf * mEf_prev);
  if (**mVr > mMaxVr)
    **mVr = mMaxVr;
  else if (**mVr < mMinVr)
    **mVr = mMinVr;

  // Exciter equation
  **mEf = Math::StateSpaceEuler(mEf_prev, -mKe / mTe, 1. / mTe, dt,
                                mVr_prev - **mVse);

  return **mEf;
}

/*
// Saturation function according to Viviane thesis
Real Signal::Exciter::saturation_fcn1(Real mVd, Real mVq, Real Vref, Real dt) {
	if (mEf <= 2.3)
		mVse = (0.1 / 2.3)*mEf;
	else
		mVse = (0.33 / 3.1)*mEf;
	mVse = mVse*mEf;
}

// Saturation function according to PSAT
Real Signal::Exciter::saturation_fcn2() {
	return mA * (exp(mB * abs(mEf)) - 1);
}

*/
