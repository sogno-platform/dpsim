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

#include <cps/EMT/EMT_Ph3_SynchronGeneratorVBR.h>

using namespace CPS;

EMT::Ph3::SynchronGeneratorVBR::SynchronGeneratorVBR(String name,
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

void EMT::Ph3::SynchronGeneratorVBR::addExciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd) {
	mExciter = Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lad, Rfd);
	mExciter.initialize(1, 1);
	WithExciter = true;
}

void EMT::Ph3::SynchronGeneratorVBR::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef) {
	mTurbineGovernor = TurbineGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
	mTurbineGovernor.initialize(PmRef, Tm_init);
	WithTurbineGovernor = true;
}

void EMT::Ph3::SynchronGeneratorVBR::initialize(Real om, Real dt,
	Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	mSystemOmega = om;
	mSystemTimeStep = dt;

	mResistanceMat = Matrix::Zero(3, 3);
	mResistanceMat <<
		mRs, 0, 0,
		0, mRs, 0,
		0, 0, mRs;

	//Dynamic mutual inductances
	mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);

	if (mNumDampingWindings == 2)
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1 + 1. / mLlkq2);
	else
	{
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1);
		K1a = Matrix::Zero(2, 1);
		K1 = Matrix::Zero(2, 1);
	}

	mLa = (mDLmq + mDLmd) / 3.;
	mLb = (mDLmd - mDLmq) / 3.;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);


	// Correcting variables
	mThetaMech = mThetaMech + PI / 2;
	mMechTorque = -mMechTorque;
	mIq = -mIq;
	mId = -mId;


	// #### VBR Model Dynamic variables #######################################
	CalculateAuxiliarConstants(dt*mBase_OmElec);

	if (mNumDampingWindings == 2)
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	else
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);

	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	mDqStatorCurrents <<
		mIq,
		mId;

	mPsikq1kq2 <<
		mPsikq1,
		mPsikq2;
	mPsifdkd <<
		mPsifd,
		mPsikd;

	CalculateAuxiliarVariables();
	K1K2 << K1, K2;
	mDVqd = K1K2*mDqStatorCurrents + h_qdr;
	mDVq = mDVqd(0);
	mDVd = mDVqd(1);

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	mDVabc <<
		mDVa,
		mDVb,
		mDVc;

	mVa = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(0);
	mVb = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(1);
	mVc = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(2);

	mIa = inverseParkTransform(mThetaMech, mIq, mId, mI0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, mI0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, mI0)(2);

	CalculateL();
}

void EMT::Ph3::SynchronGeneratorVBR::mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time) {
	stepInPerUnit(mSystemOmega, mSystemTimeStep, time, mNumericalMethod);

	if ( terminalNotGrounded(0) ) {
		Math::addToVectorElement(rightVector, simNode(0), mISourceEq(0));
	}
	if ( terminalNotGrounded(1) ) {
		Math::addToVectorElement(rightVector, simNode(1), mISourceEq(1));
	}
	if ( simNode(2) >= 0) {
		Math::addToVectorElement(rightVector, simNode(2), mISourceEq(2));
	}

	//Update Equivalent Resistance
	Math::addToMatrixElement(systemMatrix, simNode(0), simNode(0), mConductanceMat(0, 0));
	Math::addToMatrixElement(systemMatrix, simNode(0), simNode(1), mConductanceMat(0, 1));
	Math::addToMatrixElement(systemMatrix, simNode(0), simNode(2), mConductanceMat(0, 2));
	Math::addToMatrixElement(systemMatrix, simNode(1), simNode(0), mConductanceMat(1, 0));
	Math::addToMatrixElement(systemMatrix, simNode(1), simNode(1), mConductanceMat(1, 1));
	Math::addToMatrixElement(systemMatrix, simNode(1), simNode(2), mConductanceMat(1, 2));
	Math::addToMatrixElement(systemMatrix, simNode(2), simNode(0), mConductanceMat(2, 0));
	Math::addToMatrixElement(systemMatrix, simNode(2), simNode(1), mConductanceMat(2, 1));
	Math::addToMatrixElement(systemMatrix, simNode(2), simNode(2), mConductanceMat(2, 2));

	// TODO find alternative way to do this
	//system.updateLuFactored();

	if (mLogLevel != Logger::Level::off) {
		Matrix logValues(statorCurrents().rows() + dqStatorCurrents().rows() + 3, 1);
		logValues << statorCurrents()*mBase_I,  dqStatorCurrents(), electricalTorque(), rotationalSpeed(), rotorPosition();
		mLog->LogNodeValues(time, logValues);
	}
}

void EMT::Ph3::SynchronGeneratorVBR::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod) {
	// Estimate mechanical variables with euler
	if (WithTurbineGovernor == true) {
		mMechTorque = -mTurbineGovernor.step(mOmMech, 1, 300e6 / 555e6, dt);

	}

	mElecTorque = (mPsimd*mIq - mPsimq*mId);
	mOmMech = mOmMech + dt * (1. / (2. * mH) * (mElecTorque - mMechTorque));
	mThetaMech = mThetaMech + dt * (mOmMech* mBase_OmMech);


	// Calculate equivalent Resistance and current source
	mVabc <<
		mVa,
		mVb,
		mVc;

	mIabc <<
		mIa,
		mIb,
		mIc;

	mEsh_vbr = (mResistanceMat - (2 / (dt*mBase_OmElec))*mDInductanceMat)*mIabc + mDVabc - mVabc;

	CalculateL();

	mPsikq1kq2 <<
		mPsikq1,
		mPsikq2;
	mPsifdkd <<
		mPsifd,
		mPsikd;

	CalculateAuxiliarVariables();

	R_eq_vbr = mResistanceMat + (2 / (dt*mBase_OmElec))*mDInductanceMat + K;
	E_eq_vbr = mEsh_vbr + E_r_vbr;

	mConductanceMat = (R_eq_vbr*mBase_Z).inverse();
	mISourceEq = R_eq_vbr.inverse()*E_eq_vbr*mBase_I;
}

void EMT::Ph3::SynchronGeneratorVBR::mnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time) {
	if ( terminalNotGrounded(0) ) {
		mVa = Math::realFromVectorElement(leftVector, simNode(0)) / mBase_V;
	}
	else {
		mVa = 0;
	}
	if ( terminalNotGrounded(1) ) {
		mVb = Math::realFromVectorElement(leftVector, simNode(1)) / mBase_V;
	}
	else {
		mVb = 0;
	}
	if ( simNode(2) >= 0) {
		mVc = Math::realFromVectorElement(leftVector, simNode(2)) / mBase_V;
	}
	else {
		mVc = 0;
	}

	// ################ Update machine stator and rotor variables ############################
	mVabc <<
		mVa,
		mVb,
		mVc;

	mVq = parkTransform(mThetaMech, mVa, mVb, mVc)(0);
	mVd = parkTransform(mThetaMech, mVa, mVb, mVc)(1);
	mV0 = parkTransform(mThetaMech, mVa, mVb, mVc)(2);

	if (WithExciter == true) {
		mVfd = mExciter.step(mVd, mVq, 1, mSystemTimeStep);
	}

	mIabc = R_eq_vbr.inverse()*(mVabc - E_eq_vbr);

	mIa = mIabc(0);
	mIb = mIabc(1);
	mIc = mIabc(2);

	mIq_hist = mIq;
	mId_hist = mId;

	mIq = parkTransform(mThetaMech, mIa, mIb, mIc)(0);
	mId = parkTransform(mThetaMech, mIa, mIb, mIc)(1);
	mI0 = parkTransform(mThetaMech, mIa, mIb, mIc)(2);

	// Calculate rotor flux likanges
	if (mNumDampingWindings == 2) {
		mDqStatorCurrents <<
			mIq,
			mId;

		mPsikq1kq2 = E1*mIq + E2*mPsikq1kq2 + E1*mIq_hist;
		mPsifdkd = F1*mId + F2*mPsifdkd + F1*mId_hist + F3*mVfd;

		mPsikq1 = mPsikq1kq2(0);
		mPsikq2 = mPsikq1kq2(1);
		mPsifd = mPsifdkd(0);
		mPsikd = mPsifdkd(1);

	}
	else {

		mDqStatorCurrents <<
			mIq,
			mId;

		mPsikq1 = E1_1d*mIq + E2_1d*mPsikq1 + E1_1d*mIq_hist;
		mPsifdkd = F1*mId + F2*mPsifdkd + F1*mId_hist + F3*mVfd;

		mPsifd = mPsifdkd(0);
		mPsikd = mPsifdkd(1);
	}


	// Calculate dynamic flux likages
	if (mNumDampingWindings == 2) {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}

	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	K1K2 << K1, K2;
	mDVqd = K1K2*mDqStatorCurrents + h_qdr;
	mDVq = mDVqd(0);
	mDVd = mDVqd(1);

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);
	mDVabc <<
		mDVa,
		mDVb,
		mDVc;
}

void EMT::Ph3::SynchronGeneratorVBR::CalculateL() {
	mDInductanceMat <<
		mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
		-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
		-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);
}

void EMT::Ph3::SynchronGeneratorVBR::CalculateAuxiliarConstants(Real dt) {

	b11 = (mRkq1 / mLlkq1)*(mDLmq / mLlkq1 - 1);
	b13 = mRkq1*mDLmq / mLlkq1;
	b31 = (mRfd / mLlfd)*(mDLmd / mLlfd - 1);
	b32 = mRfd*mDLmd / (mLlfd*mLlkd);
	b33 = mRfd*mDLmd / mLlfd;
	b41 = mRkd*mDLmd / (mLlfd*mLlkd);
	b42 = (mRkd / mLlkd)*(mDLmd / mLlkd - 1);
	b43 = mRkd*mDLmd / mLlkd;

	c23 = mDLmd*mRfd / (mLlfd*mLlfd)*(mDLmd / mLlfd - 1) + mDLmd*mDLmd*mRkd / (mLlkd*mLlkd*mLlfd);
	c24 = mDLmd*mRkd / (mLlkd*mLlkd)*(mDLmd / mLlkd - 1) + mDLmd*mDLmd*mRfd / (mLlfd*mLlfd*mLlkd);
	c25 = (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd;
	c26 = mDLmd / mLlfd;

	if (mNumDampingWindings == 2) {
		b12 = mRkq1*mDLmq / (mLlkq1*mLlkq2);
		b21 = mRkq2*mDLmq / (mLlkq1*mLlkq2);
		b22 = (mRkq2 / mLlkq2)*(mDLmq / mLlkq2 - 1);
		b23 = mRkq2*mDLmq / mLlkq2;
		c11 = mDLmq*mRkq1 / (mLlkq1*mLlkq1)*(mDLmq / mLlkq1 - 1) + mDLmq*mDLmq*mRkq2 / (mLlkq2*mLlkq2*mLlkq1);
		c12 = mDLmq*mRkq2 / (mLlkq2*mLlkq2)*(mDLmq / mLlkq2 - 1) + mDLmq*mDLmq*mRkq1 / (mLlkq1*mLlkq1*mLlkq2);
		c15 = (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq;

		Ea <<
			2 - dt*b11, -dt*b12,
			-dt*b21, 2 - dt*b22;
		E1b <<
			dt*b13,
			dt*b23;
		E1 = Ea.inverse() * E1b;

		E2b <<
			2 + dt*b11, dt*b12,
			dt*b21, 2 + dt*b22;
		E2 = Ea.inverse() * E2b;
	}
	else {
		c11 = mDLmq*mRkq1 / (mLlkq1*mLlkq1)*(mDLmq / mLlkq1 - 1);
		c15 = (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq;

		E1_1d = (1 / (2 - dt*b11))*dt*b13;
		E2_1d = (1 / (2 - dt*b11))*(2 + dt*b11);
	}

	Fa <<
		2 - dt*b31, -dt*b32,
		-dt*b41, 2 - dt*b42;
	F1b <<
		dt*b33,
		dt*b43;
	F1 = Fa.inverse() * F1b;

	F2b <<
		2 + dt*b31, dt*b32,
		dt*b41, 2 + dt*b42;

	F2 = Fa.inverse() * F2b;

	F3b <<
		2 * dt,
		0;
	F3 = Fa.inverse()* F3b;

	C26 <<
		0,
		c26;
}

void EMT::Ph3::SynchronGeneratorVBR::CalculateAuxiliarVariables() {

	if (mNumDampingWindings == 2) {
		c21_omega = -mOmMech*mDLmq / mLlkq1;
		c22_omega = -mOmMech*mDLmq / mLlkq2;
		c13_omega = mOmMech*mDLmd / mLlfd;
		c14_omega = mOmMech*mDLmd / mLlkd;

		K1a <<
			c11, c12,
			c21_omega, c22_omega;
		K1b <<
			c15,
			0;
		K1 = K1a*E1 + K1b;
	}
	else {
		c21_omega = -mOmMech*mDLmq / mLlkq1;
		c13_omega = mOmMech*mDLmd / mLlfd;
		c14_omega = mOmMech*mDLmd / mLlkd;

		K1a <<
			c11,
			c21_omega;
		K1b <<
			c15,
			0;
		K1 = K1a*E1_1d + K1b;
	}

	K2a <<
		c13_omega, c14_omega,
		c23, c24;
	K2b <<
		0,
		c25;
	K2 = K2a*F1 + K2b;

	K <<
		K1, K2, Matrix::Zero(2, 1),
		0, 0, 0;

	mKrs_teta <<
		2. / 3. * cos(mThetaMech), 2. / 3. * cos(mThetaMech - 2. * M_PI / 3.), 2. / 3. * cos(mThetaMech + 2. * M_PI / 3.),
		2. / 3. * sin(mThetaMech), 2. / 3. * sin(mThetaMech - 2. * M_PI / 3.), 2. / 3. * sin(mThetaMech + 2. * M_PI / 3.),
		1. / 3., 1. / 3., 1. / 3.;

	mKrs_teta_inv <<
		cos(mThetaMech), sin(mThetaMech), 1.,
		cos(mThetaMech - 2. * M_PI / 3.), sin(mThetaMech - 2. * M_PI / 3.), 1,
		cos(mThetaMech + 2. * M_PI / 3.), sin(mThetaMech + 2. * M_PI / 3.), 1.;

	K = mKrs_teta_inv*K*mKrs_teta;

	if (mNumDampingWindings == 2)
		h_qdr = K1a*E2*mPsikq1kq2 + K1a*E1*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;
	else
		h_qdr = K1a*E2_1d*mPsikq1 + K1a*E1_1d*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;

	H_qdr << h_qdr,
		0;

	E_r_vbr = mKrs_teta_inv*H_qdr;
}

Matrix EMT::Ph3::SynchronGeneratorVBR::parkTransform(Real theta, Real a, Real b, Real c) {

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

Matrix EMT::Ph3::SynchronGeneratorVBR::inverseParkTransform(Real theta, Real q, Real d, Real zero) {

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
