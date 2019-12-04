/** Simplified Voltage behind reactance (EMT)
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

#include <cps/EMT/EMT_Ph3_SynchronGeneratorVBRSmpl.h>

using namespace CPS;

EMT::Ph3::SynchronGeneratorVBRSmpl::SynchronGeneratorVBRSmpl(String name,
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

void EMT::Ph3::SynchronGeneratorVBRSmpl::addExciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd)
{
	mExciter = Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lad, Rfd);
	//init exciter
	mHasExciter = true;
}

void EMT::Ph3::SynchronGeneratorVBRSmpl::initialize(Real om, Real dt,
	Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

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

	mLa = (mDLmq + mDLmd) / 3.;
	mLb = (mDLmd - mDLmq) / 3.;

	mDLd = mLl + mDLmd;
	mDLq = mLl + mDLmq;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

	// Correcting variables
	mThetaMech = mThetaMech + PI / 2;
	mMechTorque = -mMechTorque;
	mIq = -mIq;
	mId = -mId;

	// VBR Model Dynamic variables

	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
	mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);

	mDVq = mOmMech*mDPsid;
	mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);

	mDVd = -mOmMech*mDPsiq;
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	mDVqd <<
		mDVq,
		mDVd;

	mVa = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(0);
	mVb = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(1);
	mVc = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(2);

	mIa = inverseParkTransform(mThetaMech, mIq, mId, mI0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, mI0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, mI0)(2);

}

void EMT::Ph3::SynchronGeneratorVBRSmpl::mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time) {
	R_load = systemMatrix.inverse() / mBase_Z;

	stepInPerUnit(mSystemOmega, mSystemTimeStep, time, mNumericalMethod);

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

void EMT::Ph3::SynchronGeneratorVBRSmpl::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{
	// Calculate mechanical variables with euler
	//mElecTorque = (mPsimd*mIq - mPsimq*mId);
	mElecTorque = mVd*mId + mVq*mIq + mRs*(mId*mId + mIq*mIq);
	mOmMech = mOmMech + dt * (1. / (2. * mH) * (mElecTorque - mMechTorque));
	mThetaMech = mThetaMech + dt * (mOmMech* mBase_OmMech);

	Matrix R(2, 2);
	R <<
		-(mRs + R_load(0, 0)), -mOmMech*mDLd,
		mOmMech*mDLq, -(mRs + R_load(1, 1));

	mDqStatorCurrents = R.inverse()*mDVqd;

	mIq = mDqStatorCurrents(0, 0);
	mId = mDqStatorCurrents(1, 0);

	mIa_hist = mIa;
	mIb_hist = mIb;
	mIc_hist = mIc;

	mIa = inverseParkTransform(mThetaMech, mIq, mId, 0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, 0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, 0)(2);

	mIabc <<
		mIa*mBase_I,
		mIb*mBase_I,
		mIc*mBase_I;

	mVq = -mRs*mIq - mOmMech*mDLd*mId + mDVq;
	mVd = -mRs*mId + mOmMech*mDLq*mIq + mDVd;

	if (mHasExciter == true) {
		mVfd = mExciter.step(mVd, mVq, 1, dt);
	}

	C_flux <<
		0,
		0,
		mVfd,
		0;

	mRotorFlux <<
		mPsikq1,
		mPsikq2,
		mPsifd,
		mPsikd;

	mRotorFlux = Math::StateSpaceTrapezoidal(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents);

	mPsikq1 = mRotorFlux(0);
	mPsikq2 = mRotorFlux(1);
	mPsifd = mRotorFlux(2);
	mPsikd = mRotorFlux(3);

	// Calculate dynamic flux likages
	mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
	mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	// Calculate dynamic voltages
	mDVq = mOmMech*mDPsid;
	mDVd = -mOmMech*mDPsiq;

	mDVqd <<
		mDVq,
		mDVd;
}

void EMT::Ph3::SynchronGeneratorVBRSmpl::CalculateLandpL()
{
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

Matrix EMT::Ph3::SynchronGeneratorVBRSmpl::parkTransform(Real theta, Real a, Real b, Real c)
{
	Matrix dq0vector(3, 1);

	Real q, d;

	q = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	d = 2. / 3. * sin(theta)*a + 2. / 3. * sin(theta - 2. * M_PI / 3.)*b + 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	dq0vector << q,
		d,
		0;

	return dq0vector;
}

Matrix EMT::Ph3::SynchronGeneratorVBRSmpl::inverseParkTransform(Real theta, Real q, Real d, Real zero)
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
