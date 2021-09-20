/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/Exciter.h>
#include <cps/MathUtils.h>

using namespace CPS;

void Signal::Exciter::setParameters(Real Ta, Real Ka, Real Te, Real Ke,
	Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd) {
	mTa = Ta;
	mKa = Ka;
	mTe = Te;
	mKe = Ke;
	mTf = Tf;
	mKf = Kf;
	mTr = Tr;
	mLad = Lad;
	mRfd = Rfd;
}

void Signal::Exciter::initialize(Real Vh_init, Real Vf_init) {
	mVf = 1;
	mVse = mVf <= 2.3 ? 0.1 / 2.3 : 0.33 / 3.1;
	mVse *= mVf;

	mVr = mVse + mKe*mVf;
	mVf_init = mVr/mKa;
	mVh = 1;
	mVm = mVh;
	mVis = 0;
}

Real Signal::Exciter::step(Real mVd, Real mVq, Real Vref, Real dt) {
	mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
	// Voltage Transducer equation
	mVm = Math::StateSpaceEuler(mVm, -1, 1, dt / mTr, mVh);
	// Stabilizing feedback equation
	mVis = Math::StateSpaceEuler(mVis, -1, mKf, dt / mTf, ((mVr - mVse) - mVf*mKe)/mTe);
	// Amplifier equation
	mVr = Math::StateSpaceEuler(mVr, -1, mKa, dt / mTa, Vref - mVm - mVis + mVf_init);
	if (mVr > 1)
			mVr = 1;
	else if (mVr < -0.9)
			mVr = -0.9;
	// Exciter
	if (mVf <= 2.3)
			mVse = (0.1 / 2.3)*mVf;
	else
			mVse = (0.33 / 3.1)*mVf;
	mVse = mVse*mVf;
	mVf = Math::StateSpaceEuler(mVf, -mKe, 1, dt / mTe, mVr - mVse);

	return (mRfd / mLad)*mVf;
}
