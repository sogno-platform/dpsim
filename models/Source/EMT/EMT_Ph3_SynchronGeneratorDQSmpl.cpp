/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/EMT/EMT_Ph3_SynchronGeneratorDQSmpl.h>

using namespace CPS;

// !!! TODO: 	Adaptions to use in EMT_Ph3 models phase-to-ground peak variables
// !!! 			with initialization from phase-to-phase RMS variables

EMT::Ph3::SynchronGeneratorDQSmpl::SynchronGeneratorDQSmpl(String name,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, Real Ra, Logger::Level logLevel)
	: SynchronGeneratorBase(name, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logLevel) {

	mRa = Ra;
}

void EMT::Ph3::SynchronGeneratorDQSmpl::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mRa = mRa*mBase_Z;

	// Set diagonal entries
	if ( terminalNotGrounded(0) ) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(0), matrixNodeIndex(0), 1 / mRa);
	}
	if ( terminalNotGrounded(1) ) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(1), matrixNodeIndex(1), 1 / mRa);
	}
	if ( matrixNodeIndex(2) >= 0) {
		Math::addToMatrixElement(systemMatrix, matrixNodeIndex(2), matrixNodeIndex(2), 1 / mRa);
	}
}

void EMT::Ph3::SynchronGeneratorDQSmpl::initialize(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt,
	Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	mSystemOmega = om;
	mSystemTimeStep = dt;

	// Create matrices for state space representation
	mVoltages = Matrix::Zero(3, 1);
	mFluxes = Matrix::Zero(3, 1);
	mCurrents = Matrix::Zero(3, 1);
	mInductanceMat = Matrix::Zero(3, 3);
	mResistanceMat = Matrix::Zero(3, 3);
	mReactanceMat = Matrix::Zero(3, 3);
	mOmegaFluxMat = Matrix::Zero(3, 3);

	mInductanceMat <<
		-(mLl + mLmq), 0, 0,
		0, -(mLl + mLmd), mLmd,
		0, -mLmd, mLlfd + mLmd;

	mResistanceMat <<
		mRs, 0, 0,
		0, mRs, 0,
		0, 0, -mRfd;

	mOmegaFluxMat <<
		0, 1, 0,
		-1, 0, 0,
		0, 0, 0;

	// Determinant of Ld (inductance matrix of d axis)
	//detLd = -(mLmd + mLl)*(mLlfd + mLmd) + mLmd*mLmd;

	mReactanceMat = mInductanceMat.inverse();

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

	// Calculation of operational parameters
	mXd = mOmMech*(mLl + mLmd);
	mXq = mOmMech*(mLl + mLmq);
	mTd0_t = (mLlfd + mLmd) / (mRfd*mBase_OmMech);
	mXd_t = mOmMech*(mLl + mLmd - mLmd*mLmd / (mLlfd + mLmd));
	mEq_t = mVq + mXd_t*mId;
	mEf = mOmMech*(mLmd / mRfd) * mVfd;

	// mVdq0 <<
	// 	0.0017966582295287615,
	// 	0.99999836649049489,
	// 	0;

	mVa = inverseParkTransform(mThetaMech, mVd* mBase_V,  mVq* mBase_V,  mV0* mBase_v)(0);
	mVb = inverseParkTransform(mThetaMech, mVd* mBase_V,  mVq* mBase_V,  mV0* mBase_v)(1);
	mVc = inverseParkTransform(mThetaMech, mVd* mBase_V,  mVq* mBase_V,  mV0* mBase_v)(2);

	mIa = inverseParkTransform(mThetaMech, mId* mBase_I,  mIq* mBase_I,  mI0* mBase_i)(0);
	mIb = inverseParkTransform(mThetaMech, mId* mBase_I,  mIq* mBase_I,  mI0* mBase_i)(1);
	mIc = inverseParkTransform(mThetaMech, mId* mBase_I,  mIq* mBase_I,  mI0* mBase_i)(2);
}

void EMT::Ph3::SynchronGeneratorDQSmpl::mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time) {
	mG_load = systemMatrix;
	mR_load = mG_load.inverse() / mBase_Z;

	stepInPerUnit(mSystemOmega, mSystemTimeStep, time, mNumericalMethod);

	// Update current source accordingly
	if ( terminalNotGrounded(0) ) {
		Math::addToVectorElement(rightVector, matrixNodeIndex(0), mIa + mVa / mRa);
	}
	if ( terminalNotGrounded(1) ) {
		Math::addToVectorElement(rightVector, matrixNodeIndex(1), mIb + mVb / mRa);
	}
	if ( matrixNodeIndex(2) >= 0) {
		Math::addToVectorElement(rightVector, matrixNodeIndex(2), mIc + mVc / mRa);
	}

	if (mLogLevel != Logger::Level::off) {
		Matrix logValues(statorCurrents().rows() + currents().rows() + 3, 1);
		logValues << statorCurrents()*mBase_I,  currents(), electricalTorque(), rotationalSpeed(), rotorPosition();
		mLog->logNodeValues(time, logValues);
	}
}

void EMT::Ph3::SynchronGeneratorDQSmpl::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{

	// Calculation of rotational speed with euler
	mElecTorque = (mPsid*mIq - mPsiq*mId);
	mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));

	// Calculation of rotor angular position
	mThetaMech = mThetaMech + dt * (mOmMech * mBase_OmMech);


	mPsifd = mPsifd + dt*mBase_OmMech*(mVfd - mRfd*mIfd);

	mR_eq <<
		-mRs, (mLl + mLmq),
		-(mLl + mLmd) + mLmd*mLmd / (mLlfd + mLmd), -mRs;
	mE_eq <<
		0,
		(mLmd)*mPsifd / (mLlfd + mLmd);

	mKrs_teta <<
		2. / 3. * cos(mThetaMech), 2. / 3. * cos(mThetaMech - 2. * M_PI / 3.), 2. / 3. * cos(mThetaMech + 2. * M_PI / 3.),
		-2. / 3. * sin(mThetaMech), -2. / 3. * sin(mThetaMech - 2. * M_PI / 3.), -2. / 3. * sin(mThetaMech + 2. * M_PI / 3.);

	mKrs_teta_inv <<
		cos(mThetaMech), -sin(mThetaMech),
		cos(mThetaMech - 2. * M_PI / 3.), -sin(mThetaMech - 2. * M_PI / 3.),
		cos(mThetaMech + 2. * M_PI / 3.), -sin(mThetaMech + 2. * M_PI / 3.);

	mR_eq_abc = mKrs_teta_inv*mR_eq*mKrs_teta;
	mG_eq_abc = (mR_eq_abc - mR_load).inverse();
	mEq_abc = mKrs_teta_inv*mE_eq;

	//Matrix mEq_abc2 = Matrix::Zero(3, 1);
	//mEq_abc2 = inverseParkTransform(mThetaMech, mE_eq(0), mE_eq(1), 0);

	//Matrix mIdq2 = Matrix::Zero(2, 1);
	//Matrix mVdq2 = Matrix::Zero(2, 1);
	//Matrix mIabc2 = Matrix::Zero(3, 1);

	//mVdq2 << mVdq0(0),
	//	mVdq0(1);

	//mIdq2 = mR_eq.inverse()*(mVdq2 - mE_eq);

	//mIabc2 = mKrs_teta_inv*mIdq2;

	mIabc = -mG_eq_abc*mEq_abc;
	mIdq = mKrs_teta*mIabc;

	//mVdq = mR_eq*mIdq + mE_eq;

	mId = mIdq(0);
	mIq = mIdq(1);
	mIfd = (mPsifd + mLmd*mId) / (mLlfd + mLmd);

	mCurrents(0, 0) = mIq;
	mCurrents(1, 0) = mId;
	mCurrents(2, 0) = mIfd;

	mFluxes = mInductanceMat*mCurrents;

	mPsiq = mFluxes(0, 0);
	mPsid = mFluxes(1, 0);
	mPsifd = mFluxes(2, 0);

	mIa = mBase_I *  mIabc(0);
	mIb = mBase_I *  mIabc(1);
	mIc = mBase_I *  mIabc(2);

	mVoltages <<
		mVq,
		mVd,
		mVfd;
}

void EMT::Ph3::SynchronGeneratorDQSmpl::mnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time) {
	if ( terminalNotGrounded(0) ) {
		mVa = Math::realFromVectorElement(leftVector, matrixNodeIndex(0));
	}
	else {
		mVa = 0;
	}

	if ( terminalNotGrounded(1) ) {
		mVb = Math::realFromVectorElement(leftVector, matrixNodeIndex(1));
	}
	else {
		mVb = 0;
	}

	if ( matrixNodeIndex(2) >= 0) {
		mVc = Math::realFromVectorElement(leftVector, matrixNodeIndex(2));
	}
	else {
		mVc = 0;
	}

}

Matrix EMT::Ph3::SynchronGeneratorDQSmpl::parkTransform(Real theta, Real a, Real b, Real c)
{
	Matrix dq0vector(3, 1);

	// Park transform according to Kundur
	Real d, q;

	d = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	q = -2. / 3. * sin(theta)*a - 2. / 3. * sin(theta - 2. * M_PI / 3.)*b - 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	Vector zero;
	zero << 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector <<
		d,
		q,
		0;

	return dq0vector;
}

Matrix EMT::Ph3::SynchronGeneratorDQSmpl::inverseParkTransform(Real theta, Real d, Real q, Real zero)
{
	Matrix abcVector(3, 1);

	// Park transform according to Kundur
	Real a, b, c;

	a = cos(theta)*d - sin(theta)*q + 1.*zero;
	b = cos(theta - 2. * M_PI / 3.)*d - sin(theta - 2. * M_PI / 3.)*q + 1.*zero;
	c = cos(theta + 2. * M_PI / 3.)*d - sin(theta + 2. * M_PI / 3.)*q + 1.*zero;

	abcVector <<
		a,
		b,
		c;

	return abcVector;
}
