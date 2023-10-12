/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/ExciterST1Simp.h>
#include <dpsim-models/MathUtils.h>

using namespace CPS;

Signal::ExciterST1Simp::ExciterST1Simp(const String & name, CPS::Logger::Level logLevel) 
	: SimSignalComp(name, name, logLevel) { 

	this->setExciterType(ExciterType::ST1Simp);
}
	
void Signal::ExciterST1Simp::setParameters(std::shared_ptr<Base::ExciterParameters> parameters) {
	
	if (auto temp_struct = std::dynamic_pointer_cast<Signal::ExciterST1Parameters>(parameters)){
		mTr = temp_struct->Tr;
		mKa = temp_struct->Ka;

		SPDLOG_LOGGER_INFO(mSLog,
			"Exciter ST1Simp parameters:"
			"\nType: ST1Simp"
			"\nTr: {:e}"
			"\nKa: {:e}\n",
			mTr, mKa);
	} else {
		std::cout << "The type of the ExciterParameters of " << this->name() << " has to be ExciterST1Parameters!" << std::endl;
		throw CPS::TypeException();
	}		
}

void Signal::ExciterST1Simp::initialize(Real Vh_init, Real Ef_init) {
	
	mVh = Vh_init;
	mEf = Ef_init;

	SPDLOG_LOGGER_INFO(mSLog,
		"Initially set excitation system initial values:"
		"\ninit Vh: {:e}"
		"\ninit Ef: {:e}",
		mVh, mEf);

	/// init value of transducer output
	mVr = mVh;
	
	/// exciter reference
	mVref = mVr - mEf / mKa;

	SPDLOG_LOGGER_INFO(mSLog,
		"Actually applied excitation system initial values:"
		"\nVref : {:e}"
		"\ninit Vr: {:e}",
		mVref, mVr);
	mSLog->flush();
}

Real Signal::ExciterST1Simp::step(Real Vd, Real Vq, Real dt, Real Vpss) {
	// Voltage magnitude calculation
	mVh = sqrt(pow(Vd, 2.) + pow(Vq, 2.));

	// update state variables at time k-1
	mVr_prev = mVr;

	// compute state variables at time k using euler forward

	// Voltage Transducer
	mVr = mVr_prev + dt / mTr * (mVh - mVr_prev);
	
	// Exciter output
	mEf =(mVr + Vpss - mVref) * mKa; 
	
	return mEf;
}