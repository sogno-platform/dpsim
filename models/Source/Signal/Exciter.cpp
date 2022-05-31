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
	Vm(Attribute<Real>::create("Vm", mAttributes, 0)),
	Vr(Attribute<Real>::create("Vr", mAttributes, 0)),
	Vf(Attribute<Real>::create("Vf", mAttributes, 0)),
	Vis(Attribute<Real>::create("Vis", mAttributes, 0)),
	Vse(Attribute<Real>::create("Vse", mAttributes, 0)) { }
}

void Signal::Exciter::setParameters(Real Ta, Real Ka, Real Te, Real Ke,
	Real Tf, Real Kf, Real Tr, Real maxVr, Real minVr) {
	mTa = Ta;
	mKa = Ka;
	mTe = Te;
	mKe = Ke;
	mTf = Tf;
	mKf = Kf;
	mTr = Tr;
	mMaxVr = maxVr;
	mMinVr = minVr;

	mSLog->info("Exciter parameters: \n"
				"Ta: {:e}"
				"\nKa: {:e}"
				"\nTe: {:e}"
				"\nKe: {:e}"
				"\nTf: {:e}"
				"\nKf: {:e}"
				"\nTr: {:e}"
				"\nMaximum regulator Voltage: {:e}"
				"\nMinimum regulator Voltage: {:e}\n",
				mTa, mKa, 
				mTe, mKe,
				mTf, mKf,
				mTr,
				mMaxVr,
				mMinVr);
}

void Signal::Exciter::initialize(Real Vh_init, Real Vf_init) {
	
	mSLog->info("Initially set excitation system initial values: \n"
				"Vh_init: {:e}\nVf_init: {:e}\n",
				Vh_init, Vf_init);

	**mVm = Vh_init;
	**mVf = Vf_init;

	// mVse is the ceiling function in PSAT
	// mVse = mVf * (0.33 * (exp(0.1 * abs(mVf)) - 1.));
	**mVse = **mVf * (0.33 * exp(0.1 * abs(**mVf)));

	// mVis = vr2 in PSAT
	**mVis = - mKf / mTf * **mVf;

	// mVr = vr1 in PSAT
	**mVr = mKe * **mVf + **mVse;
	if (**mVr > mMaxVr)
		**mVr = mMaxVr;
	else if (**mVr < mMinVr)
		**mVr = mMinVr;

	mVref = **mVr /  mKa + **mVm;
	mSLog->info("Actually applied excitation system initial values:"
				"\nVref : {:e}"
				"\ninit_Vm: {:e}"
				"\ninit_Vf: {:e}"
				"\ninit_Vc: {:e}"
				"\ninit_Vr: {:e}"				
				"\ninit_Vr2: {:e}",
				mVref,
				**mVm, 
				**mVf,
				**mVse, 
				**mVr,
				**mVis);
}

Real Signal::Exciter::step(Real mVd, Real mVq, Real dt) {
	// Voltage magnitude calculation
	**mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));

	// update state variables at time k-1
	mVm_prev = **mVm;
	mVis_prev = **mVis;
	mVr_prev = **mVr;
	mVf_prev = **mVf;

	// compute state variables at time k using euler forward

	// Voltage Transducer equation
	**mVm = Math::StateSpaceEuler(mVm_prev, -1 / mTr, 1 / mTr, dt, **mVh);

	// Stabilizing feedback equation
	// mVse = mVf * (0.33 * (exp(0.1 * abs(mVf)) - 1.));
	**mVse = mVf_prev * (0.33 * exp(0.1 * abs(mVf_prev)));
	**mVis = Math::StateSpaceEuler(mVis_prev, -1 / mTf, -mKf / mTf / mTf, dt, mVf_prev);

	// Voltage regulator equation
	**mVr = Math::StateSpaceEuler(mVr_prev, -1 / mTa, mKa / mTa, dt, mVref - **mVm - mVis_prev - mKf / mTf * mVf_prev);
	if (**mVr > mMaxVr)
		**mVr = mMaxVr;
	else if (**mVr < mMinVr)
		**mVr = mMinVr;

	// Exciter equation
	**mVf = Math::StateSpaceEuler(mVf_prev, - mKe / mTe, 1. / mTe, dt, mVr_prev - **mVse);
	
	return **mVf;
}

/*
// Saturation function according to Viviane thesis
Real Signal::Exciter::saturation_fcn1(Real mVd, Real mVq, Real Vref, Real dt) {
	if (mVf <= 2.3)
		mVse = (0.1 / 2.3)*mVf;
	else
		mVse = (0.33 / 3.1)*mVf;
	mVse = mVse*mVf;
}

// Saturation function according to PSAT
Real Signal::Exciter::saturation_fcn2() {
	return mA * (exp(mB * abs(mVf)) - 1);
}

*/