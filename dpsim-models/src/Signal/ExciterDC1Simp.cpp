/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/ExciterDC1Simp.h>

using namespace CPS;
using namespace CPS::Signal;

ExciterDC1Simp::ExciterDC1Simp(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { }

void ExciterDC1Simp::setParameters(std::shared_ptr<Base::ExciterParameters> parameters) {

	if (auto params = std::dynamic_pointer_cast<Signal::ExciterDC1SimpParameters>(parameters)){
		mParameters = params;

		SPDLOG_LOGGER_INFO(mSLog, 
			"\nExciterDC1Simp parameters:"
			"\nTa: {:e}"
			"\nKa: {:e}"
			"\nTef: {:e}"
			"\nKef: {:e}"
			"\nTf: {:e}"
			"\nKf: {:e}"
			"\nTr: {:e}"
			"\nAef: {:e}"
			"\nBef: {:e}"
			"\nMaximum amplifier Voltage: {:e}"
			"\nMinimum amplifier Voltage: {:e}\n",
			mParameters->Ta, mParameters->Ka, 
			mParameters->Tef, mParameters->Kef,
			mParameters->Tf, mParameters->Kf,
			mParameters->Tr, mParameters->Aef, 
			mParameters->Bef, mParameters->MaxVa, mParameters->MinVa);
	} else {
		std::cout << "Type of parameters class of " << this->name() << " has to be ExciterDC1Simp!" << std::endl;
		throw CPS::TypeException();
	}
}

void ExciterDC1Simp::initialize(Real Vh_init, Real Ef_init) {
	//
	mVh = Vh_init;
	mEf = Ef_init;
	SPDLOG_LOGGER_INFO(mSLog, 
		"Initially set excitation system initial values:"
		"\ninit Vh: {:e}"
		"\ninit Ef: {:e}",
		mVh, mEf);

	/// init value of transducer output
	mVr = mVh;

	/// init value of stabilizing feedback output
	mVf = 0.0;

	/// ceiling function
	mVsat = mParameters->Aef * exp(mParameters->Bef * abs(mEf));

	/// init value of amplifier output
	mVa = mParameters->Kef * mEf + mVsat * mEf;
	if (mVa>mParameters->MaxVa)
		mVa = mParameters->MaxVa;
	if (mVa<mParameters->MinVa)
		mVa = mParameters->MinVa;

	/// init value of amplifier input 
	mVin = mVa / mParameters->Ka;

	///
	mVref = mVr + mVin;

	/// check initial conditions
	if (mEf - mVa / (mVsat +mParameters->Kef))
		SPDLOG_LOGGER_WARN(mSLog, "\nInitial conditions are not consistent!!!");
	
	SPDLOG_LOGGER_INFO(mSLog, 
		"\nActually applied excitation system initial values:"
		"\nVref : {:e}"
		"\ninit_Vr: {:e}"
		"\ninit_Ef: {:e}"
		"\ninit_Va: {:e}",
		mVref, mVr, 
		mEf, mVa);
	mSLog->flush();
}

Real ExciterDC1Simp::step(Real mVd, Real mVq, Real dt, Real Vpss) {
	// Voltage magnitude calculation
	mVh = sqrt(pow(mVd, 2.) + pow(mVq, 2.));

	// update state variables at time k-1
	mVr_prev = mVr;
	mVa_prev = mVa;
	mVf_prev = mVf;
	mEf_prev = mEf;

	// compute state variables at time k using euler forward

	// saturation function
	mVsat = mParameters->Aef * exp(mParameters->Bef * abs(mEf_prev));

	// Voltage Transducer equation
	mVr = mVr_prev + dt / mParameters->Tr * (mVh - mVr_prev);

	// Voltage amplifier equation
	mVin = mVref + Vpss - mVr_prev - mVf_prev;
	mVa = mVa_prev + dt / mParameters->Ta * (mVin * mParameters->Ka - mVa_prev);
	if (mVa > mParameters->MaxVa)
		mVa = mParameters->MaxVa;
	else if (mVa < mParameters->MinVa)
		mVa = mParameters->MinVa;

	// Stabilizing feedback
	mVf = (1. - dt / mParameters->Tf) * mVf_prev + dt * mParameters->Kf / (mParameters->Tf * mParameters->Tef) * (mVa_prev - (mVsat + mParameters->Kef) * mEf_prev);
	
	// Exciter output
	mEf = mEf_prev + dt / mParameters->Tef * (mVa_prev - (mVsat + mParameters->Kef) * mEf_prev); 

	return mEf;
}