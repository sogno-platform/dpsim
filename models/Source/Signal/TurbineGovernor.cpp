/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/TurbineGovernor.h>
#include <cps/MathUtils.h>

using namespace CPS;

void Signal::TurbineGovernor::setParameters(Real Ta, Real Tb, Real Tc, Real Fa,
	Real Fb, Real Fc, Real K, Real Tsr, Real Tsm) {
	mTa = Ta;
	mTb = Tb;
	mTc = Tc;
	mFa = Fa;
	mFb = Fb;
	mFc = Fc;
	mK = K;
	mTsr = Tsr;
	mTsm = Tsm;
}

void Signal::TurbineGovernor::initialize(Real PmRef, Real Tm_init) {
	mTm = Tm_init;
	mVcv = PmRef;
	mpVcv = 0;
	Psm_in = PmRef;
	T1 = (1 - mFa)*PmRef;
	T2 = mFa*PmRef;
}

Real Signal::TurbineGovernor::step(Real Om, Real OmRef, Real PmRef, Real dt) {
	// ### Governing ###
	// Input of speed relay
	Psr_in = PmRef + (OmRef - Om)*mK;
	// Input of servor motor
	Psm_in = Math::StateSpaceEuler(Psm_in, -1, 1, dt / mTsr, Psr_in);
	// rate of change of valve
	mpVcv = (Psm_in - mVcv) / mTsm;
	if (mpVcv >= 0.1)
		mpVcv = 0.1;
	else if (mpVcv <= -1)
		mpVcv = -1;
	//Valve position
	mVcv = mVcv + dt*mpVcv;
	if (mVcv >= 1)
		mVcv = 1;
	else if (mVcv <= 0)
		mVcv = 0;

	//### Turbine ###
	// Simplified equation

	T1 = Math::StateSpaceEuler(T1, -1, (1 - mFa), dt / mTa, mVcv);
	T2 = mVcv*mFa;
	mTm = T1 + T2;

	return mTm;
}
