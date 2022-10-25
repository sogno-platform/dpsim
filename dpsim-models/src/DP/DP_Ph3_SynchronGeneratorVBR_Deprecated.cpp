/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/DP/DP_Ph3_SynchronGeneratorVBR.h>

using namespace CPS;

DP::Ph3::SynchronGeneratorVBR::SynchronGeneratorVBR(String name,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, Logger::Level logLevel)
	/// FIXME: SynchronGeneratorBase does not exist!
	: SynchronGeneratorBase(name, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logLevel)
{
	///CHECK: Are all of these used in this class or in subclasses?
	mRs = Attribute<Real>::create("Rs", mAttributes, 0);
	mLl = Attribute<Real>::create("Ll", mAttributes, 0);
	mOmMech = Attribute<Real>::create("w_r", mAttributes, 0);
	mElecTorque = Attribute<Real>::create("T_e", mAttributes, 0);
	mMechTorque = Attribute<Real>::create("T_m", mAttributes, 0);
}

void DP::Ph3::SynchronGeneratorVBR::addExciter(
	std::shared_ptr<Base::Exciter> exciter) {
	mExciter = exciter;
	mHasExciter = true;
}

void DP::Ph3::SynchronGeneratorVBR::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef)
{
	mTurbineGovernor = TurbineGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
	mTurbineGovernor.initialize(PmRef, Tm_init);
	mHasTurbineGovernor = true;
}

void DP::Ph3::SynchronGeneratorVBR::initialize(Real om, Real dt,
	Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle, Real initMechPower) {

	mSystemOmega = om;
	mSystemTimeStep = dt;

	mResistanceMat = Matrix::Zero(3, 3);
	mResistanceMat <<
		**mRs, 0, 0,
		0, **mRs, 0,
		0, 0, **mRs;

	//Dynamic mutual inductances
	mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);

	if (mNumDampingWindings == 2) {
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1 + 1. / mLlkq2);
	}
	else {
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1);
		K1a = Matrix::Zero(2, 1);
		K1 = Matrix::Zero(2, 1);
	}

	mLa = (mDLmq + mDLmd) / 3.;
	mLb = (mDLmd - mDLmq) / 3.;

	LD0 <<
		(**mLl + mLa), -mLa / 2, -mLa / 2,
		-mLa / 2, **mLl + mLa, -mLa / 2,
		-mLa / 2, -mLa / 2, **mLl + mLa;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initMechPower);

	mThetaMech2 = mThetaMech + PI / 2;
	mThetaMech = mThetaMech + PI / 2;
	mTheta0 = mThetaMech2;
	**mMechTorque = - **mMechTorque;
	mIq = -mIq;
	mId = -mId;

	CalculateAuxiliarConstants(dt*mBase_OmElec);

	if (mNumDampingWindings == 2) {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}

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

	CalculateAuxiliarVariables(0);

	E_r_vbr_DP = dq0ToAbcTransform(mThetaMech, h_qdr(1), h_qdr(0), 0);

	K_DP << K, Matrix::Zero(3, 3),
			Matrix::Zero(3, 3), K;

	mVabc = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0);
	mIabc = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0);

	mDVabc = K_DP*mIabc + E_r_vbr_DP + E_r_vbr_DP2;

	CalculateLandR(0, dt*mBase_OmElec);
}

void DP::Ph3::SynchronGeneratorVBR::mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time) {
	stepInPerUnit(mSystemOmega, mSystemTimeStep, time, mNumericalMethod);

	// Update current source accordingly
	if ( terminalNotGrounded(0) ) {
		Math::addToVectorElement(rightVector, matrixNodeIndex(0), Complex(mISourceEq(0), mISourceEq(3)));
	}
	if ( terminalNotGrounded(1) ) {
		Math::addToVectorElement(rightVector, matrixNodeIndex(1), Complex(mISourceEq(1), mISourceEq(4)));
	}
	if ( matrixNodeIndex(2) >= 0) {
		Math::addToVectorElement(rightVector, matrixNodeIndex(2), Complex(mISourceEq(2), mISourceEq(5)));
	}

	//Update Equivalent Resistance
	Int systemSize = systemMatrix.rows();
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), mConductanceMat(0, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1), mConductanceMat(0, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(2), mConductanceMat(0, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0), mConductanceMat(1, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), mConductanceMat(1, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(2), mConductanceMat(1, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2), matrixNodeIndex(0), mConductanceMat(2, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2), matrixNodeIndex(1), mConductanceMat(2, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2), matrixNodeIndex(2), mConductanceMat(2, 2));

	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0) + systemSize, matrixNodeIndex(0), mConductanceMat(3, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0) + systemSize, matrixNodeIndex(1), mConductanceMat(3, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0) + systemSize, matrixNodeIndex(2), mConductanceMat(3, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1) + systemSize, matrixNodeIndex(0), mConductanceMat(4, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1) + systemSize, matrixNodeIndex(1), mConductanceMat(4, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1) + systemSize, matrixNodeIndex(2), mConductanceMat(4, 2));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2) + systemSize, matrixNodeIndex(0), mConductanceMat(5, 0));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2) + systemSize, matrixNodeIndex(1), mConductanceMat(5, 1));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2) + systemSize, matrixNodeIndex(2), mConductanceMat(5, 2));

	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0) + systemSize, mConductanceMat(0, 3));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(1) + systemSize, mConductanceMat(0, 4));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(2) + systemSize, mConductanceMat(0, 5));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(0) + systemSize, mConductanceMat(1, 3));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1) + systemSize, mConductanceMat(1, 4));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(2) + systemSize, mConductanceMat(1, 5));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2), matrixNodeIndex(0) + systemSize, mConductanceMat(2, 3));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2), matrixNodeIndex(1) + systemSize, mConductanceMat(2, 4));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2), matrixNodeIndex(2) + systemSize, mConductanceMat(2, 5));

	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0) + systemSize, matrixNodeIndex(0) + systemSize, mConductanceMat(3, 3));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0) + systemSize, matrixNodeIndex(1) + systemSize, mConductanceMat(3, 4));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0) + systemSize, matrixNodeIndex(2) + systemSize, mConductanceMat(3, 5));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1) + systemSize, matrixNodeIndex(0) + systemSize, mConductanceMat(4, 3));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1) + systemSize, matrixNodeIndex(1) + systemSize, mConductanceMat(4, 4));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1) + systemSize, matrixNodeIndex(2) + systemSize, mConductanceMat(4, 5));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2) + systemSize, matrixNodeIndex(0) + systemSize, mConductanceMat(5, 3));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2) + systemSize, matrixNodeIndex(1) + systemSize, mConductanceMat(5, 4));
	Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2) + systemSize, matrixNodeIndex(2) + systemSize, mConductanceMat(5, 5));

	// TODO find solution without SystemModel
	// system.updateLuFactored();


	if (mLogLevel != Logger::Level::off) {
		Matrix logValues(statorCurrents().rows() + dqStatorCurrents().rows() + 3, 1);
		logValues << statorCurrents()*mBase_I,  dqStatorCurrents(), electricalTorque(), rotationalSpeed(), rotorPosition();
		SPDLOG_LOGGER_DEBUG(mLog, time, logValues);
	}
}

void DP::Ph3::SynchronGeneratorVBR::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{

	// Calculate mechanical variables with euler
	if (mHasTurbineGovernor == true) {
		** = -mTurbineGovernor.step(**mOmMech, 1, 300e6 / 555e6, dt);
	}

	**mElecTorque = (mPsimd*mIq - mPsimq*mId);
	**mOmMech = **mOmMech + dt * (1. / (2. * mH) * (**mElecTorque - **mMechTorque));
	mThetaMech = mThetaMech + dt * ((**mOmMech - 1) * mBase_OmMech);
	mThetaMech2 = mThetaMech2 + dt * (**mOmMech* mBase_OmMech);

	mPsikq1kq2 <<
		mPsikq1,
		mPsikq2;
	mPsifdkd <<
		mPsifd,
		mPsikd;

	CalculateLandR(time, dt*mBase_OmElec);

	CalculateAuxiliarVariables(time);


	E_r_vbr_DP = dq0ToAbcTransform(mThetaMech, h_qdr(1), h_qdr(0), 0);
	K_DP << K, Matrix::Zero(3, 3),
		Matrix::Zero(3, 3), K;

	R_eq_DP = Var1 + K_DP;
	E_eq_DP = Var2*mIabc + mDVabc - mVabc + E_r_vbr_DP + E_r_vbr_DP2;

	mConductanceMat = (R_eq_DP*mBase_Z).inverse();
	mISourceEq = R_eq_DP.inverse()*E_eq_DP*mBase_I;
}

void DP::Ph3::SynchronGeneratorVBR::mnaCompPostStep(Matrix& rightVector, Matrix& leftVector, Real time) {
	Real dt = mSystemTimeStep;

	if ( terminalNotGrounded(0) ) {
		mVaRe = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0)).real();
		mVaIm = Math::complexFromVectorElement(leftVector, matrixNodeIndex(0)).imag();
	}
	else {
		mVaRe = 0;
		mVaIm = 0;
	}
	if ( terminalNotGrounded(1) ) {
		mVbRe = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1)).real();
		mVbIm = Math::complexFromVectorElement(leftVector, matrixNodeIndex(1)).imag();
	}
	else {
		mVbRe = 0;
		mVbIm = 0;
	}
	if ( matrixNodeIndex(2) >= 0) {
		mVcRe = Math::complexFromVectorElement(leftVector, matrixNodeIndex(2)).real();
		mVcIm = Math::complexFromVectorElement(leftVector, matrixNodeIndex(2)).imag();
	}
	else {
		mVcRe = 0;
		mVcIm = 0;
	}

	mVabc <<
		mVaRe / mBase_V,
		mVbRe / mBase_V,
		mVcRe / mBase_V,
		mVaIm / mBase_V,
		mVbIm / mBase_V,
		mVcIm / mBase_V;

	mVq = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(0);
	mVd = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(1);
	mV0 = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(2);

	if (mHasExciter == true) {
		mVfd = mExciter.step(mVd, mVq, 1, dt);
	}

	mIabc = R_eq_DP.inverse()*(mVabc - E_eq_DP);

	mIaRe = mIabc(0);
	mIbRe = mIabc(1);
	mIcRe = mIabc(2);
	mIaIm = mIabc(3);
	mIbIm = mIabc(4);
	mIcIm = mIabc(5);
	mIq_hist = mIq;
	mId_hist = mId;
	mIq = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(0);
	mId = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(1);
	mI0 = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(2);

	// Calculate rotor flux likanges
	if (mNumDampingWindings == 2)
	{

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

	mDVabc = K_DP*mIabc + E_r_vbr_DP + E_r_vbr_DP2;
}

void DP::Ph3::SynchronGeneratorVBR::CalculateAuxiliarVariables(Real time) {

	if (mNumDampingWindings == 2) {
		c21_omega = - **mOmMech*mDLmq / mLlkq1;
		c22_omega = - **mOmMech*mDLmq / mLlkq2;
		c13_omega = **mOmMech*mDLmd / mLlfd;
		c14_omega = **mOmMech*mDLmd / mLlkd;

		K1a <<
			c11, c12,
			c21_omega, c22_omega;
		K1b <<
			c15,
			0;
		K1 = K1a*E1 + K1b;
	}
	else {
		c21_omega = - **mOmMech*mDLmq / mLlkq1;
		c13_omega = **mOmMech*mDLmd / mLlfd;
		c14_omega = **mOmMech*mDLmd / mLlkd;

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
		2. / 3. * cos(mThetaMech2), 2. / 3. * cos(mThetaMech2 - 2. * M_PI / 3.), 2. / 3. * cos(mThetaMech2 + 2. * M_PI / 3.),
		2. / 3. * sin(mThetaMech2), 2. / 3. * sin(mThetaMech2 - 2. * M_PI / 3.), 2. / 3. * sin(mThetaMech2 + 2. * M_PI / 3.),
		1. / 3., 1. / 3., 1. / 3.;

	// mKrs_teta_inv = mKrs_teta.inverse();
	mKrs_teta_inv <<
		cos(mThetaMech2), sin(mThetaMech2), 1.,
		cos(mThetaMech2 - 2. * M_PI / 3.), sin(mThetaMech2 - 2. * M_PI / 3.), 1,
		cos(mThetaMech2 + 2. * M_PI / 3.), sin(mThetaMech2 + 2. * M_PI / 3.), 1.;

	K = mKrs_teta_inv*K*mKrs_teta;

	if (mNumDampingWindings == 2) {
		//h_qdr = K1a*E2*mPsikq1kq2 + K1a*E1*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;
				h_qdr = K1a*E2*mPsikq1kq2 + K2a*F2*mPsifdkd + (K2a*F3 + C26)*mVfd;
	}
	else {
		h_qdr = K1a*E2_1d*mPsikq1 + K1a*E1_1d*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;
	}

	Matrix Knew = Matrix::Zero(3, 3);
	Knew <<
			K1a*E1, K2a*F1, Matrix::Zero(2, 1),
			0, 0, 0;
	Knew = mKrs_teta_inv*Knew*mKrs_teta;

	Matrix KDPnew = Matrix::Zero(6, 6);
	KDPnew << Knew, Matrix::Zero(3, 3),
			Matrix::Zero(3, 3), Knew;

	E_r_vbr_DP2 = KDPnew*mIabc;
}

void DP::Ph3::SynchronGeneratorVBR::CalculateAuxiliarConstants(Real dt) {
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

void DP::Ph3::SynchronGeneratorVBR::CalculateLandR(Real time, Real dt)
{
	Matrix L1_Re(3, 3);
	Matrix L1_Im(3, 3);
	Matrix Re_R(3, 3);
	Matrix Im_R(3, 3);
	Matrix Re_L(3, 3);
	Matrix Im_L(3, 3);
	Matrix Re_R2(3, 3);
	Matrix Im_R2(3, 3);
	Matrix Re_L2(3, 3);
	Matrix Im_L2(3, 3);

	Real b_Re = cos(2 * **mOmMech* mBase_OmMech*time);
	Real b_Im = sin(2 * **mOmMech* mBase_OmMech*time);
	Real c_Re = cos(2 * mThetaMech2 - 2 * 1 * mBase_OmMech*time - 2 * mTheta0);
	Real c_Im = sin(2 * mThetaMech2 - 2 * 1 * mBase_OmMech*time - 2 * mTheta0);

	Real a = 2 * (mTheta0);

	L1_Re <<
		cos(a), cos(-2.*PI / 3 + a), cos(2.*PI / 3 + a),
		cos(-2 * PI / 3 + a), cos(-4 * PI / 3 + a), cos(a),
		cos(2 * PI / 3 + a), cos(a), cos(4 * PI / 3 + a);
	L1_Re = mLb*L1_Re;
	L1_Im <<
		sin(a), sin(-2.*PI / 3 + a), sin(2.*PI / 3 + a),
		sin(-2 * PI / 3 + a), sin(-4 * PI / 3 + a), sin(a),
		sin(2 * PI / 3 + a), sin(a), sin(4 * PI / 3 + a);
	L1_Im = mLb*L1_Im;

	Re_R = mResistanceMat + (2 * **mOmMech + 1) / 2. * (L1_Re*b_Im + L1_Im*b_Re);
	Im_R = LD0 - (2 * **mOmMech + 1) / 2. *(L1_Re*b_Re - L1_Im*b_Im);
	Re_L = LD0 - 1. / 2.*(L1_Re*b_Re - L1_Im*b_Im);
	Im_L = -1. / 2.*(L1_Re*b_Im + L1_Im*b_Re);
	Re_R2 = 1. / 2.*(2 * **mOmMech - 1)*(L1_Im*c_Re + L1_Re*c_Im);
	Im_R2 = -1. / 2.*(2 * **mOmMech - 1)*(L1_Re*c_Re - L1_Im*c_Im);
	Re_L2 = -1. / 2.*(L1_Re*c_Re - L1_Im*c_Im);
	Im_L2 = -1. / 2.*(L1_Im*c_Re + L1_Re*c_Im);

	R_EQ <<
		Re_R + Re_R2, -Im_R + Im_R2,
		Im_R + Im_R2, Re_R - Re_R2;
	L_EQ <<
		Re_L + Re_L2, -Im_L + Im_L2,
		Im_L + Im_L2, Re_L - Re_L2;

	A = -L_EQ.inverse()*R_EQ;
	B = L_EQ.inverse();
	Var1 = B.inverse()*((2 / dt) * Matrix::Identity(6, 6) - A);
	Var2 =  -B.inverse()*((2 / dt) * Matrix::Identity(6, 6) + A);
}

Matrix DP::Ph3::SynchronGeneratorVBR::abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm)
{
	// Balanced case
	Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));
	Complex thetaCompInv(cos(-theta), sin(-theta));
	MatrixComp AbcToPnz(3, 3);
	AbcToPnz <<
		1, 1, 1,
		1, alpha, pow(alpha, 2),
		1, pow(alpha, 2), alpha;
	AbcToPnz = (1. / 3.) * AbcToPnz;

	MatrixComp abcVector(3, 1);
	abcVector <<
		Complex(aRe, aIm),
		Complex(bRe, bIm),
		Complex(cRe, cIm);

	MatrixComp pnzVector(3, 1);
	pnzVector = AbcToPnz * abcVector * thetaCompInv;

	Matrix dq0Vector(3, 1);
	dq0Vector <<
		pnzVector(1, 0).real(),
		-pnzVector(1, 0).imag(),
		0;

	return dq0Vector;
}

Matrix DP::Ph3::SynchronGeneratorVBR::dq0ToAbcTransform(Real theta, Real d, Real q, Real zero)
{
	// Balanced case
	Complex alpha(cos(2. / 3. * PI), sin(2. / 3. * PI));
	Complex thetaComp(cos(theta), sin(theta));
	MatrixComp PnzToAbc(3, 3);
	PnzToAbc <<
		1, 1, 1,
		1, pow(alpha, 2), alpha,
		1, alpha, pow(alpha, 2);

	MatrixComp pnzVector(3, 1);
	pnzVector <<
		0,
		Complex(q, -d),
		Complex(0, 0);

	MatrixComp abcCompVector(3, 1);
	abcCompVector = PnzToAbc * pnzVector * thetaComp;

	Matrix abcVector(6, 1);
	abcVector <<
		abcCompVector(0, 0).real(),
		abcCompVector(1, 0).real(),
		abcCompVector(2, 0).real(),
		abcCompVector(0, 0).imag(),
		abcCompVector(1, 0).imag(),
		abcCompVector(2, 0).imag();

	return abcVector;
}

