/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/TurbineGovernorType1.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;
using namespace CPS::Signal;

Signal::TurbineGovernorType1::TurbineGovernorType1(
	const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel){ }

void TurbineGovernorType1::setParameters(Real T3, Real T4, Real T5, 
	Real Tc, Real Ts, Real R, Real Pmin, Real Pmax, Real OmRef) {
	mT3 = T3;
	mT4 = T4;
	mT5 = T5;
	mTc = Tc;
	mTs = Ts;
	mR = R;
	mPmin = Pmin;
	mPmax = Pmax;
	mOmRef = OmRef;

	SPDLOG_LOGGER_INFO(mSLog, "TurbineGovernorType1 parameters: "
				"\nT3: {:e}"
				"\nT4: {:e}"
				"\nT5: {:e}"
				"\nTc: {:e}"
				"\nTs: {:e}"
				"\nR: {:e}"
				"\nTmin: {:e}"
				"\nTmax: {:e}"
				"\nOmRef: {:e}",
				mT3, mT4, mT5,
				mTc, mTs, mR,
				mPmin, mPmax, mOmRef);
}

void TurbineGovernorType1::initialize(Real PmRef) {
	mPmRef = PmRef;
	mXg1 = PmRef;
	mXg2 = (1 - mT3 / mTc) * mXg1;
	mXg3 = (1 - mT4 / mT5) * (mXg2 + mT3 / mTc * mXg1);
	mTm = mXg3 + mT4 / mT5 * (mXg2 + mT3 / mTc * mXg1);

	SPDLOG_LOGGER_INFO(mSLog, "Governor initial values: \n"
				"\nTorder: {:f}"
				"\nXg1: {:f}"
				"\nXg2: {:f}"
				"\nXg3: {:f}"
				"\nTm: {:f}",
				mPmRef, mXg1, mXg2, mXg3, mTm);
}

Real TurbineGovernorType1::step(Real Omega, Real dt) {
	
	/// update state variables at time k-1
	mXg1_prev = mXg1;
	mXg2_prev = mXg2;
	mXg3_prev = mXg3;

	/// Input of speed relay
	Real Pin = mPmRef + (mOmRef - Omega) / mR;
	if (Pin>mPmax)
		Pin = mPmax;
	if (Pin<mPmin)
		Pin = mPmin;
	
	/// Governor
	mXg1 = mXg1_prev + dt / mTs * (Pin - mXg1_prev);

	/// Servo
	mXg2 = mXg2_prev + dt / mTc * ((1 - mT3 / mTc) * mXg1_prev - mXg2_prev);

	/// Reheat
	mXg3 = mXg3_prev + dt / mT5 * ((1- mT4 / mT5) * (mXg2_prev + mT3 / mTc * mXg1_prev) - mXg3_prev);

	/// Mechanical torque
	mTm = mXg3 + mT4 / mT5 * (mXg2 + mT3 / mTc * mXg1);

	return mTm;
}
