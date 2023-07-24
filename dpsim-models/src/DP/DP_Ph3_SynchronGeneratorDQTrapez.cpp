/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_SynchronGeneratorDQTrapez.h>

using namespace CPS;

DP::Ph3::SynchronGeneratorDQTrapez::SynchronGeneratorDQTrapez(String uid, String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(uid, name, logLevel) {
}

DP::Ph3::SynchronGeneratorDQTrapez::SynchronGeneratorDQTrapez(String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(name, name, logLevel) {
}

void DP::Ph3::SynchronGeneratorDQTrapez::mnaCompInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
		updateMatrixNodeIndices();
	mTimeStep = timeStep;

	SPDLOG_LOGGER_DEBUG(mSLog,
		"\nFluxStateSpaceMat: \n{}"
		"\nOmegaFluxMat: \n{}"
		"\nResistances: {} {} {}",
		mFluxStateSpaceMat,
		mOmegaFluxMat,
		mVdq0(0,0)/mIdq0(0,0),mVdq0(1,0)/mIdq0(1,0),mVdq0(2,0)/mIdq0(2,0));
}

void DP::Ph3::SynchronGeneratorDQTrapez::mnaCompAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	modifiedAttributes.push_back(mRightVector);
	prevStepDependencies.push_back(mIntfVoltage);
}

void DP::Ph3::SynchronGeneratorDQTrapez::mnaCompPreStep(Real time, Int timeStepCount) {
	stepInPerUnit(time); //former system solve (trapezoidal)
	mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::SynchronGeneratorDQTrapez::setMultisamplingRate(Int rate) {
			mMultisamplingRate = rate;
}

void DP::Ph3::SynchronGeneratorDQTrapez::stepInPerUnit(Real time) {
	// Receives the voltage of time step k and has to
	// calculate the current output for time step k+1
	for (Int i = 0; i < mMultisamplingRate; i++) {
	// Calculate per unit values and
	// transform per unit voltages from abc to dq0
	mVdq0 = abcToDq0Transform(mThetaMech, **mIntfVoltage);
	mVdq0 = mVdq0 / mBase_V;
	mVsr(0,0) = mVdq0(0,0);
	mVsr(3,0) = mVdq0(1,0);
	mVsr(6,0) = mVdq0(2,0);
	// Current is already available in pu dq0 from previous step

	// Update of fd winding voltage from exciter
	if (mHasExciter) {
		//mVsr(1,0) = mExciter.step(mVsr(0,0), mVsr(3,0), 1, mTimeStep);
	}

	// Update of mechanical torque from turbine governor
	if (mHasTurbineGovernor == true) {
		//mMechTorque = mTurbineGovernor.step(mOmMech, 1, 300e6 / 555e6, mTimeStep);
	}

	// Calculation of electrical torque
	**mElecTorque = (mPsisr(3,0)*mIsr(0,0) - mPsisr(0,0)*mIsr(3,0));

	// Update mechanical rotor angle with respect to electrical angle
	// using Euler and previous states
	mThetaMech = mThetaMech + mTimeStep * ((**mOmMech - 1) * mBase_OmMech);

	// Update of omega using Euler
	**mOmMech = **mOmMech + mTimeStep * (1./(2.* **mInertia) * (**mMechTorque - **mElecTorque));

	// Update of fluxes
	if (mNumericalMethod == NumericalMethod::Euler) {
		mPsisr = Math::StateSpaceEuler(mPsisr,
			mBase_OmElec*(mFluxStateSpaceMat + mOmegaFluxMat* **mOmMech),
			mBase_OmElec*mVsr, mTimeStep / mMultisamplingRate);
	} else {
		mPsisr = Math::StateSpaceTrapezoidal(mPsisr,
			mBase_OmElec*(mFluxStateSpaceMat + mOmegaFluxMat* **mOmMech),
			mBase_OmElec*mVsr, mTimeStep / mMultisamplingRate);
	}

	// Calculate new currents from fluxes
	mIsr = mFluxToCurrentMat * mPsisr;
	}

	mIdq0(0,0) = mIsr(0,0);
	mIdq0(1,0) = mIsr(3,0);
	mIdq0(2,0) = mIsr(6,0);
	**mIntfCurrent = mBase_I * dq0ToAbcTransform(mThetaMech, mIdq0);
}
