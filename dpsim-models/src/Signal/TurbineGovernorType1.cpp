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

void TurbineGovernorType1::setParameters(std::shared_ptr<Base::GovernorParameters> parameters) {

	if (auto params = std::dynamic_pointer_cast<Signal::TurbineGovernorType1Parameters>(parameters)){
		mParameters = params;
		SPDLOG_LOGGER_INFO(mSLog, 
			"TurbineGovernorType1 parameters: "
			"\nT3: {:e}"
			"\nT4: {:e}"
			"\nT5: {:e}"
			"\nTc: {:e}"
			"\nTs: {:e}"
			"\nR: {:e}"
			"\nTmin: {:e}"
			"\nTmax: {:e}"
			"\nOmRef: {:e}",
			mParameters->T3, mParameters->T4, mParameters->T5,
			mParameters->Tc, mParameters->Ts, mParameters->R,
			mParameters->Pmin, mParameters->Pmax, mParameters->OmRef);
	} else {
		std::cout << "The type of the pparameter GovernorParameters of " << this->name() << " has to be TurbineGovernorType1Parameters!" << std::endl;
		throw CPS::TypeException();
	}
}

void TurbineGovernorType1::initialize(Real PmRef) {
	mPmRef = PmRef;
	mXg1 = PmRef;
	mXg2 = (1 - mParameters->T3 / mParameters->Tc) * mXg1;
	mXg3 = (1 - mParameters->T4 / mParameters->T5) * (mXg2 + mParameters->T3 / mParameters->Tc * mXg1);
	mTm = mXg3 + mParameters->T4 / mParameters->T5 * (mXg2 + mParameters->T3 / mParameters->Tc * mXg1);

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
	Real Pin = mPmRef + (mParameters->OmRef - Omega) / mParameters->R;
	if (Pin>mParameters->Pmax)
		Pin = mParameters->Pmax;
	if (Pin<mParameters->Pmin)
		Pin = mParameters->Pmin;
	
	/// Governor
	mXg1 = mXg1_prev + dt / mParameters->Ts * (Pin - mXg1_prev);

	/// Servo
	mXg2 = mXg2_prev + dt / mParameters->Tc * ((1 - mParameters->T3 / mParameters->Tc) * mXg1_prev - mXg2_prev);

	/// Reheat
	mXg3 = mXg3_prev + dt / mParameters->T5 * ((1- mParameters->T4 / mParameters->T5) * (mXg2_prev + mParameters->T3 / mParameters->Tc * mXg1_prev) - mXg3_prev);

	/// Mechanical torque
	mTm = mXg3 + mParameters->T4 / mParameters->T5 * (mXg2 + mParameters->T3 / mParameters->Tc * mXg1);

	return mTm;
}
