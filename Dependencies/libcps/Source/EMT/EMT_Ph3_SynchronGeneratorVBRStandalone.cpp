/** Voltage behind reactance (EMT)
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/EMT/EMT_Ph3_SynchronGeneratorVBRStandalone.h>

using namespace CPS;

EMT::Ph3::SynchronGeneratorVBRStandalone::SynchronGeneratorVBRStandalone(String name,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, Logger::Level logLevel)
	: SynchronGeneratorBase(name, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logLevel)
{
}

void EMT::Ph3::SynchronGeneratorVBRStandalone::addExciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd)
{
	mExciter = Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lad, Rfd);
	mExciter.initialize(1,1);

	mHasExciter = true;
}

void EMT::Ph3::SynchronGeneratorVBRStandalone::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef)
{
	mTurbineGovernor = TurbineGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
	mTurbineGovernor.initialize(PmRef, Tm_init);
	mHasTurbineGovernor = true;
}

void EMT::Ph3::SynchronGeneratorVBRStandalone::initialize(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt,
	Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	mSystemOmega = om;
	mSystemTimeStep = dt;

	mResistanceMat = Matrix::Zero(3, 3);
	mResistanceMat <<
		mRs, 0, 0,
		0, mRs, 0,
		0, 0, mRs;

	R_load <<
		1037.8378 / mBase_Z, 0, 0,
		0, 1037.8378 / mBase_Z, 0,
		0, 0, 1037.8378 / mBase_Z;

	//Dynamic mutual inductances
	mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);
	if (mNumDampingWindings == 2) {
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1 + 1. / mLlkq2);

		A_flux <<
			-mRkq1 / mLlkq1*(1 - mDLmq / mLlkq1), mRkq1 / mLlkq1*(mDLmq / mLlkq2), 0, 0,
			mRkq2 / mLlkq2*(mDLmq / mLlkq1), -mRkq2 / mLlkq2*(1 - mDLmq / mLlkq2), 0, 0,
			0, 0, -mRfd / mLlfd*(1 - mDLmd / mLlfd), mRfd / mLlfd*(mDLmd / mLlkd),
			0, 0, mRkd / mLlkd*(mDLmd / mLlfd), -mRkd / mLlkd*(1 - mDLmd / mLlkd);
		B_flux <<
			mRkq1*mDLmq / mLlkq1, 0,
			mRkq2*mDLmq / mLlkq2, 0,
			0, mRfd*mDLmd / mLlfd,
			0, mRkd*mDLmd / mLlkd;
	}
	else {
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1);

		A_flux = Matrix::Zero(3, 3);
		B_flux = Matrix::Zero(3, 2);
		C_flux = Matrix::Zero(3, 1);
		mRotorFlux = Matrix::Zero(3, 1);
		mDqStatorCurrents = Matrix::Zero(2, 1);
		mDqStatorCurrents_hist = Matrix::Zero(2, 1);

		A_flux <<
			-mRkq1 / mLlkq1*(1 - mDLmq / mLlkq1), 0, 0,
			0, -mRfd / mLlfd*(1 - mDLmd / mLlfd), mRfd / mLlfd*(mDLmd / mLlkd),
			0, mRkd / mLlkd*(mDLmd / mLlfd), -mRkd / mLlkd*(1 - mDLmd / mLlkd);
		B_flux <<
			mRkq1*mDLmq / mLlkq1, 0,
			0, mRfd*mDLmd / mLlfd,
			0, mRkd*mDLmd / mLlkd;
	}

	mLa = (mDLmq + mDLmd) / 3.;
	mLb = (mDLmd - mDLmq) / 3.;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

	// Correcting variables
	mThetaMech = mThetaMech + PI/2;
	mMechTorque = -mMechTorque;
	mIq = -mIq;
	mId = -mId;

	// VBR Model Dynamic variables
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);

	if (mNumDampingWindings == 2) {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
			mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1);
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) + (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq*mIq;
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}

	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	mDVabc <<
		mDVa,
		mDVb,
		mDVc;
	mDVabc_hist = mDVabc;

	mVa = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(0);
	mVb = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(1);
	mVc = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(2);

	mIa = inverseParkTransform(mThetaMech, mIq, mId, mI0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, mI0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, mI0)(2);
}

void EMT::Ph3::SynchronGeneratorVBRStandalone::mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time) {
	stepInPerUnit(mSystemOmega, mSystemTimeStep, time, mNumericalMethod);

	R_load = systemMatrix.inverse() / mBase_Z;

	// Update current source accordingly
	if ( terminalNotGrounded(0) ) {
		Math::addToVectorElement(rightVector, simNode(0), -mIa*mBase_i);
	}
	if ( terminalNotGrounded(1) ) {
		Math::addToVectorElement(rightVector, simNode(1), -mIb*mBase_i);
	}
	if ( simNode(2) >= 0) {
		Math::addToVectorElement(rightVector, simNode(2), -mIc*mBase_i);
	}

	if (mLogLevel != Logger::Level::off) {
		Matrix logValues(rotorFluxes().rows() + dqStatorCurrents().rows() + 3, 1);
		logValues << rotorFluxes(), dqStatorCurrents(), electricalTorque(), rotationalSpeed(), rotorPosition();
		mLog->debug(time, logValues);
	}
}

void EMT::Ph3::SynchronGeneratorVBRStandalone::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{
	mIabc <<
		mIa,
		mIb,
		mIc;

	// Calculate mechanical variables with euler
	if (mHasTurbineGovernor == true) {
		mMechTorque = -mTurbineGovernor.step(mOmMech, 1, 0.001, dt);
	}

	mElecTorque = (mPsimd*mIq - mPsimq*mId);
	mOmMech = mOmMech + dt * (1. / (2. * mH) * (mElecTorque - mMechTorque));
	mThetaMech = mThetaMech + dt * (mOmMech* mBase_OmMech);

	// Calculate Inductance matrix and its derivative
	CalculateLandpL();

	// Solve circuit - calculate stator currents
	if (numMethod == NumericalMethod::Trapezoidal_flux) {
		mIabc = Math::StateSpaceTrapezoidal(mIabc, -mDInductanceMat.inverse()*(mResistanceMat + R_load + pmDInductanceMat), mDInductanceMat.inverse(), dt*mBase_OmElec, -mDVabc, -mDVabc_hist);

	}

	else if (numMethod == NumericalMethod::Euler) {
		mIabc = Math::StateSpaceEuler(mIabc, -mDInductanceMat.inverse()*(mResistanceMat + R_load + pmDInductanceMat), mDInductanceMat.inverse(), dt*mBase_OmElec, -mDVabc);
	}

	mIa_hist = mIa;
	mIb_hist = mIb;
	mIc_hist = mIc;

	mIa = mIabc(0);
	mIb = mIabc(1);
	mIc = mIabc(2);
	Real mIq_hist = mIq;
	Real mId_hist = mId;
	mIq = parkTransform(mThetaMech, mIa, mIb, mIc)(0);
	mId = parkTransform(mThetaMech, mIa, mIb, mIc)(1);
	mI0 = parkTransform(mThetaMech, mIa, mIb, mIc)(2);

	if (mHasExciter == true) {
		// dq-transform of interface voltage
		mVd = parkTransform(mThetaMech, mVa / mBase_V,  mVb / mBase_V,  mVc / mBase_v)(0);
		mVq = parkTransform(mThetaMech, mVa / mBase_V,  mVb / mBase_V,  mVc / mBase_v)(1);
		mV0 = parkTransform(mThetaMech, mVa / mBase_V,  mVb / mBase_V,  mVc / mBase_v)(2);
		mVfd = mExciter.step(mVd, mVq, 1, dt);
	}

	// Calculate rotor flux likanges
	if (mNumDampingWindings == 2) {
		C_flux <<
			0,
			0,
			mVfd,
			0;

		mDqStatorCurrents <<
			mIq,
			mId;

		mDqStatorCurrents_hist <<
			mIq_hist,
			mId_hist;

		mRotorFlux <<
			mPsikq1,
			mPsikq2,
			mPsifd,
			mPsikd;

		if (numMethod == NumericalMethod::Trapezoidal_flux)
			mRotorFlux = Math::StateSpaceTrapezoidal(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents, mDqStatorCurrents_hist);
		else if (numMethod == NumericalMethod::Euler)
			mRotorFlux = Math::StateSpaceEuler(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents);

		mPsikq1 = mRotorFlux(0);
		mPsikq2 = mRotorFlux(1);
		mPsifd = mRotorFlux(2);
		mPsikd = mRotorFlux(3);
	}
	else {
		C_flux <<
			0,
			mVfd,
			0;

		mDqStatorCurrents <<
			mIq,
			mId;
		mDqStatorCurrents_hist <<
			mIq_hist,
			mId_hist;

		mRotorFlux <<
			mPsikq1,
			mPsifd,
			mPsikd;

		if (numMethod == NumericalMethod::Trapezoidal_flux)
			mRotorFlux = Math::StateSpaceTrapezoidal(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents, mDqStatorCurrents_hist);
		else if (numMethod == NumericalMethod::Euler)
			mRotorFlux = Math::StateSpaceEuler(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents);

		mPsikq1 = mRotorFlux(0);
		mPsifd = mRotorFlux(1);
		mPsikd = mRotorFlux(2);
	}

	// Calculate dynamic flux likages
	if (mNumDampingWindings == 2) {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1);
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	// Calculate dynamic voltages
	if (mNumDampingWindings == 2) {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
			mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	}
	else {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) + (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq*mIq;
	}
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);
	mDVabc_hist = mDVabc;
	mDVabc <<
		mDVa,
		mDVb,
		mDVc;
}

void EMT::Ph3::SynchronGeneratorVBRStandalone::mnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time) {
	if ( terminalNotGrounded(0) ) {
		mVa = Math::realFromVectorElement(leftVector, simNode(0));
	}
	else {
		mVa = 0;
	}
	if ( terminalNotGrounded(1) ) {
		mVb = Math::realFromVectorElement(leftVector, simNode(1));
	}
	else {
		mVb = 0;
	}
	if ( simNode(2) >= 0) {
		mVc = Math::realFromVectorElement(leftVector, simNode(2));
	}
	else {
		mVc = 0;
	}
}

void EMT::Ph3::SynchronGeneratorVBRStandalone::CalculateLandpL() {
	mDInductanceMat <<
		mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
		-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
		-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);
	pmDInductanceMat <<
		mLb*sin(2 * mThetaMech), mLb*sin(2 * mThetaMech - 2 * PI / 3), mLb*sin(2 * mThetaMech + 2 * PI / 3),
		mLb*sin(2 * mThetaMech - 2 * PI / 3), mLb*sin(2 * mThetaMech - 4 * PI / 3), mLb*sin(2 * mThetaMech),
		mLb*sin(2 * mThetaMech + 2 * PI / 3), mLb*sin(2 * mThetaMech), mLb*sin(2 * mThetaMech + 4 * PI / 3);
	pmDInductanceMat = pmDInductanceMat * 2 * mOmMech;
}

Matrix EMT::Ph3::SynchronGeneratorVBRStandalone::parkTransform(Real theta, Real a, Real b, Real c)
{
	Matrix dq0vector(3, 1);

	Real q, d, zero;

	q = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	d = 2. / 3. * sin(theta)*a + 2. / 3. * sin(theta - 2. * M_PI / 3.)*b + 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;
	zero = 1. / 3.*a + 1. / 3.*b + 1. / 3.*c;

	dq0vector << q,
		d,
		zero;

	return dq0vector;
}

Matrix EMT::Ph3::SynchronGeneratorVBRStandalone::inverseParkTransform(Real theta, Real q, Real d, Real zero)
{
	Matrix abcVector(3, 1);

	Real a, b, c;

	a = cos(theta)*q + sin(theta)*d + 1.*zero;
	b = cos(theta - 2. * M_PI / 3.)*q + sin(theta - 2. * M_PI / 3.)*d + 1.*zero;
	c = cos(theta + 2. * M_PI / 3.)*q + sin(theta + 2. * M_PI / 3.)*d + 1.*zero;

	abcVector << a,
		b,
		c;

	return abcVector;
}
