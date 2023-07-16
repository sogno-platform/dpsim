/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/PSS1A.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

Signal::PSS1A::PSS1A(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel),
	mV1(mAttributes->create<Real>("V1")),
	mV2(mAttributes->create<Real>("V2")),
	mV3(mAttributes->create<Real>("V3")),
	mVs(mAttributes->create<Real>("Vs")) { }
	
void Signal::PSS1A::setParameters(Real Kp, Real Kv, Real Kw, Real T1, 
		Real T2, Real T3, Real T4, Real Vs_max, Real Vs_min, Real Tw, Real dt) {

	mKp = Kp;
	mKv = Kv;
	mKw = Kw;
	mT1 = T1;
	mT2 = T2;
	mT3 = T3;
	mT4 = T4;
	mVs_max = Vs_max;
	mVs_min = Vs_min;
	mTw = Tw;
	mTimeStep = dt;

	mA = 1. - mT1 / mT2;
	mB = 1. - mT3 / mT4;

	SPDLOG_LOGGER_INFO(mSLog,
		"PSS Type2 parameters: \n"
		"Kp: {:e}"
		"\nKv: {:e}"
		"\nKw: {:e}"
		"\nT1: {:e}"
		"\nT2: {:e}"
		"\nT3: {:e}"
		"\nT4: {:e}"
		"\nMaximum stabiliter output signal: {:e}"
		"\nMinimum stabiliter output signal: {:e}"
		"\nTw: {:e}"
		"\nStep size: {:e}\n",
		Kp, Kv, Kw,
		mT1, mT2, mT1, mT4,
		mVs_max, mVs_min,
		mTw, mTimeStep);
}

void Signal::PSS1A::initialize(Real omega, Real activePower, Real Vd, Real Vq) {

	// Voltage magnitude calculation
	Real Vh = sqrt(pow(Vd, 2.) + pow(Vq, 2.));

	**mV1 = - (mKw * omega + mKp * activePower + mKv * Vh);
	**mV2 = mA * (mKw * omega + mKp * activePower + mKv * Vh + **mV1);
	**mV3 = mB * (**mV2 + (mT1 / mT2) * (mKw * omega + mKp * activePower + mKv * Vh + **mV1));
	**mVs = **mV3 + (mT3 / mT4) * (**mV2 + (mT1 / mT2) * (mKw * omega + mKp * activePower + mKv * Vh + **mV1));

	mOmega_prev = omega;

	SPDLOG_LOGGER_INFO(mSLog,
		"Initial values: "
		"\nmV1(t=0): {:e}"
		"\nmV2(t=0): {:e}"
		"\nmV3(t=0): {:e}"
		"\nmVs(t=0): {:e}",
		**mV1, **mV2, **mV3, **mVs);
	mSLog->flush();
}

Real Signal::PSS1A::step(Real omega, Real activePower, Real Vd, Real Vq) {

	// Voltage magnitude calculation
	Real Vh = sqrt(pow(Vd, 2.) + pow(Vq, 2.));

	// 
	mV1_prev = **mV1;
	mV2_prev = **mV2;
	mV3_prev = **mV3;
	mVs_prev = **mVs;

	// compute state variables at time k using euler forward
	**mV1 = mV1_prev - mTimeStep / mTw * (mKw * mOmega_prev + mKp * activePower + mKv * Vh + mV1_prev);
	**mV2 = mV2_prev + mTimeStep / mT2 * (mA * (mKw * mOmega_prev + mKp * activePower + mKv * Vh + mV1_prev) - mV2_prev);
	**mV3 = mV3_prev + mTimeStep / mT4 * (mB * (mV2_prev + (mT1 / mT2) * (mKw * mOmega_prev + mKp * activePower + mKv * Vh + mV1_prev)) - mV3_prev); 	

	//
	**mVs = **mV3 + (mT3 / mT4) * (**mV2 + (mT1 / mT2)*(mKw * omega + mKp * activePower + mKv * Vh + **mV1));
	mOmega_prev = omega;

	// check limints of mVs
	if (**mVs > mVs_max) {
		**mVs = mVs_max;
	} else if (**mVs < mVs_min) { 
		**mVs = mVs_min;
	}

	return **mVs;
}