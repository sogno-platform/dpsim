/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <cps/DP/DP_Ph3_SynchronGeneratorDQTrapez.h>

using namespace CPS;

DP::Ph3::SynchronGeneratorDQTrapez::SynchronGeneratorDQTrapez(String uid, String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(uid, name, logLevel) {
}

DP::Ph3::SynchronGeneratorDQTrapez::SynchronGeneratorDQTrapez(String name, Logger::Level logLevel)
	: SynchronGeneratorDQ(name, name, logLevel) {
}

void DP::Ph3::SynchronGeneratorDQTrapez::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	mTimeStep = timeStep;

	mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));

	mSLog->info(
		"\nFluxStateSpaceMat: \n{}"
		"\nOmegaFluxMat: \n{}"
		"\nResistances: {} {} {}",
		mFluxStateSpaceMat,
		mOmegaFluxMat,
		mVdq0(0,0)/mIdq0(0,0),mVdq0(1,0)/mIdq0(1,0),mVdq0(2,0)/mIdq0(2,0));
}

void DP::Ph3::SynchronGeneratorDQTrapez::MnaPreStep::execute(Real time, Int timeStepCount) {
	mSynGen.stepInPerUnit(time); //former system solve (trapezoidal)
	mSynGen.mnaApplyRightSideVectorStamp(mSynGen.mRightVector);
}

void DP::Ph3::SynchronGeneratorDQTrapez::stepInPerUnit(Real time) {
	// Receives the voltage of time step k and has to
	// calculate the current output for time step k+1
	for (Int i = 0; i < mMultisamplingRate; i++) {
	// Calculate per unit values and
	// transform per unit voltages from abc to dq0
	mVdq0 = abcToDq0Transform(mThetaMech, mIntfVoltage);
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
	mElecTorque = (mPsisr(3,0)*mIsr(0,0) - mPsisr(0,0)*mIsr(3,0));

	// Update mechanical rotor angle with respect to electrical angle
	// using Euler and previous states
	mThetaMech = mThetaMech + mTimeStep * ((mOmMech - 1) * mBase_OmMech);

	// Update of omega using Euler
	mOmMech = mOmMech + mTimeStep * (1./(2.*mInertia) * (mMechTorque - mElecTorque));

	// Update of fluxes
	if (mNumericalMethod == NumericalMethod::Euler) {
		mPsisr = Math::StateSpaceEuler(mPsisr,
			mBase_OmElec*(mFluxStateSpaceMat + mOmegaFluxMat*mOmMech),
			mBase_OmElec*mVsr, mTimeStep / mMultisamplingRate);
	} else {
		mPsisr = Math::StateSpaceTrapezoidal(mPsisr,
			mBase_OmElec*(mFluxStateSpaceMat + mOmegaFluxMat*mOmMech),
			mBase_OmElec*mVsr, mTimeStep / mMultisamplingRate);
	}

	// Calculate new currents from fluxes
	mIsr = mFluxToCurrentMat * mPsisr;
	}

	mIdq0(0,0) = mIsr(0,0);
	mIdq0(1,0) = mIsr(3,0);
	mIdq0(2,0) = mIsr(6,0);
	mIntfCurrent = mBase_I * dq0ToAbcTransform(mThetaMech, mIdq0);
}
