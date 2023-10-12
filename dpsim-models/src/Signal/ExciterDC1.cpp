/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/ExciterDC1.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

Signal::ExciterDC1::ExciterDC1(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { 

	this->setExciterType(ExciterType::DC1);
}
	
void Signal::ExciterDC1::setParameters(std::shared_ptr<Base::ExciterParameters> parameters) {
	
	
	if (auto temp_struct = std::dynamic_pointer_cast<Signal::ExciterDC1Parameters>(parameters)){
		mTr = temp_struct->Tr;
		mTa = temp_struct->Ta;
		mTb = temp_struct->Tb;
		mTc = temp_struct->Tc;
		mTef = temp_struct->Tef;
		mTf = temp_struct->Tf;
		mKa = temp_struct->Ka;
		mKef = temp_struct->Kef;
		mKf = temp_struct->Kf;
		mAef = temp_struct->Aef;
		mBef = temp_struct->Bef;
		mMaxVa = temp_struct->MaxVa;
		mMinVa = temp_struct->MinVa;
		
		SPDLOG_LOGGER_INFO(mSLog,
			"ExciterDC1 parameters:"
			"\nType: DC1"
			"\nTr: {:e}"
			"\nTa: {:e}"
			"\nKa: {:e}"
			"\nTb: {:e}"
			"\nTc: {:e}"
			"\nTef: {:e}"
			"\nKef: {:e}"
			"\nTf: {:e}"
			"\nKf: {:e}"
			"\nAef: {:e}"
			"\nBef: {:e}"
			"\nMaximum amplifier Voltage: {:e}"
			"\nMinimum amplifier Voltage: {:e}\n",
			mTr,
			mTa, mKa, 
			mTb, mTc,
			mTef, mKef,
			mTf, mKf,
			mAef, mBef,
			mMaxVa, mMinVa);
	} else {
		std::cout << "The type of the ExciterParameters of " << this->name() << " has to be ExciterParametersDC1!" << std::endl;
		throw CPS::TypeException();
	}
}

void Signal::ExciterDC1::initialize(Real Vh_init, Real Ef_init) {
	
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

	// ceiling function
	mVsat = mAef * exp(mBef * abs(mEf));

	/// init value of amplifier output
	mVa = mKef * mEf + mVsat * mEf;
	if (mVa>mMaxVa)
		mVa = mMaxVa;
	if (mVa<mMinVa)
		mVa = mMinVa;

	/// init value of amplifier input
	mVin = mVa / mKa;

	/// 
	mVb = mVin;
	
	/// exciter reference
	mVref = mVr + mVin;
	
	/// check initial conditions	
	if (mEf - mVa / (mVsat + mKef))
		SPDLOG_LOGGER_WARN(mSLog,"Initial conditions are not consistent!!!");

	SPDLOG_LOGGER_INFO(mSLog,
		"Actually applied excitation system initial values:"
		"\nVref : {:e}"
		"\ninit Vr: {:e}"
		"\ninit Vf: {:e}"
		"\ninit Vb: {:e}"
		"\ninit Vin: {:e}"				
		"\ninit Va: {:e}",
		mVref,
		mVr, 
		mVf,
		mVb, 
		mVin,
		mVa);
	mSLog->flush();
}

Real Signal::ExciterDC1::step(Real Vd, Real Vq, Real dt, Real Vpss) {
	// Voltage magnitude calculation
	mVh = sqrt(pow(Vd, 2.) + pow(Vq, 2.));

	// update state variables at time k-1
	mVr_prev = mVr;
	mVf_prev = mVf;
	mVb_prev = mVb;
	mVin_prev = mVin;
	mVa_prev = mVa;
	mEf_prev = mEf;

	// compute state variables at time k using euler forward

	// saturation function
	mVsat = mAef * exp(mBef * abs(mEf_prev));

	// Voltage Transducer
	mVr = mVr_prev + dt / mTr * (mVh - mVr_prev);

	// Regulator output
	mVb = mVb_prev * (1 - dt / mTb) + dt / mTb * (mVref + Vpss - mVr_prev - mVf_prev);
	mVin = dt * (mTc / mTb) * (mVref + Vpss - mVr_prev - mVf_prev - mVb_prev) + mVb;

	// Amplifier
	mVa = mVa_prev + dt / mTa * (mVin_prev * mKa - mVa_prev);
	if (mVa>mMaxVa)
		mVa = mMaxVa;
	if (mVa<mMinVa)
		mVa = mMinVa;

	// Stabilizing feedback
	mVf = (1. - dt / mTf) * mVf_prev + dt * mKf / (mTf * mTef) * (mVa_prev - (mVsat + mKef) * mEf_prev);
		
	// Exciter output
	mEf = mEf_prev + dt / mTef * (mVa_prev - (mVsat + mKef) * mEf_prev); 
	
	return mEf;
}