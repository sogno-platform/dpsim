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

Signal::Exciter::Exciter(String name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel),
	mVh(Attribute<Real>::create("Vh", mAttributes, 0)),
	mVr(Attribute<Real>::create("Vr", mAttributes, 0)),
	mVf(Attribute<Real>::create("Vf", mAttributes, 0)) { }

void Signal::Exciter::setParameters(Real Ta, Real Ka, Real Te, Real Ke,
	Real Tf, Real Kf, Real Tr) {
	mTa = Ta;
	mKa = Ka;
	mTe = Te;
	mKe = Ke;
	mTf = Tf;
	mKf = Kf;
	mTr = Tr;

	mSLog->info("Exciter parameters: \n"
				"Ta: {:e}\nKa: {:e}\n"
				"Te: {:e}\nKe: {:e}\n"
				"Tf: {:e}\nKf: {:e}\n"
				"Tr: {:e}\n",
				mTa, mKa, 
				mTe, mKe,
				mTf, mKf,
				mTr);
}

void Signal::Exciter::initialize(Real Vh_init, Real Vf_init) {
	
	mSLog->info("Initially set excitation system initial values: \n"
				"Vh_init: {:e}\nVf_init: {:e}\n",
				Vh_init, Vf_init);

	**mVf = Vf_init;
	mVse = **mVf <= 2.3 ? 0.1 / 2.3 : 0.33 / 3.1;
	mVse *= **mVf;

	**mVr = mVse + mKe* **mVf;
	mVf_init = **mVr / mKa;

	**mVh = Vh_init;
	mVm = **mVh;
	mVis = 0;

	mSLog->info("Actually applied excitation system initial values: \n"
				"init_Vf: {:e}\ninit_Vse: {:e}\n"
				"init_Vr: {:e}\ninit_Vf_init: {:e}\n"
				"init_Vh: {:e}\ninit_Vm: {:e}\ninit_Vis: {:e}\n",
				**mVf, mVse, 
				**mVr, mVf_init,
				**mVh, mVm, mVis);
}

Real Signal::Exciter::step(Real mVd, Real mVq, Real Vref, Real dt) {
	// Voltage magnitude calculation
	**mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));
	
	// Voltage Transducer equation
	mVm = Math::StateSpaceEuler(mVm, -1 / mTr, 1 / mTr, dt, **mVh);
	
	// Stabilizing feedback equation
	mVis = Math::StateSpaceEuler(mVis, -1 / mTf, mKf / mTe / mTf, dt, **mVr - mVse - **mVf * mKe);

	// Voltage regulator equation
	**mVr = Math::StateSpaceEuler(**mVr, -1 / mTa, mKa / mTa, dt, Vref - mVm - mVis + mVf_init);

	// Voltage regulator limiter
	// Vr,max and Vr,min parameters 
	// from M. Eremia, "Handbook of Electrical Power System Dynamics", 2013, p.96
	// TODO: fix hard-coded limiter values
	if (**mVr > 1)
		**mVr = 1;
	else if (**mVr < -0.9)
		**mVr = -0.9;
	
	// Exciter saturation
	// Se(Ef1), Ef1, Se(Ef2) and Ef2
	// from M. Eremia, "Handbook of Electrical Power System Dynamics", 2013, p.96
	// TODO: fix hard-coded saturation values
	if (**mVf <= 2.3)
		mVse = (0.1 / 2.3) * **mVf;
	else
		mVse = (0.33 / 3.1) * **mVf;
	mVse = mVse * **mVf;

	// Exciter equation
	**mVf = Math::StateSpaceEuler(**mVf, - mKe / mTe, 1 / mTe, dt, **mVr - mVse);

	return **mVf;
}
