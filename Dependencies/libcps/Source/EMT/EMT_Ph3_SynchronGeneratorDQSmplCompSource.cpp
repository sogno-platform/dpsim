/** Simplified Synchron generator (EMT)
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

#include <cps/EMT/EMT_Ph3_SynchronGeneratorDQSmplCompSource.h>

using namespace CPS;

EMT::Ph3::SynchronGeneratorDQSmplCompSource::SynchronGeneratorDQSmplCompSource(String name,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, Logger::Level logLevel)
	: SynchronGeneratorBase(name, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logLevel),
		// TODO add virtual nodes
		va("va", "va", mVa, logLevel),
		vb("vb", "vb", mVb, logLevel),
		vc("vb", "vc", mVc, logLevel)
{
	mNumVirtualNodes = 3;
	mVirtualNodes = { 0, 0, 0 };
}

void EMT::Ph3::SynchronGeneratorDQSmplCompSource::initialize(Real om, Real dt,
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

	mReactanceMat = mInductanceMat.inverse();

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);
	//mMechTorque = -mMechTorque;

	// Calculation of operational parameters
	mXd = mOmMech*(mLl + mLmd);
	mXq = mOmMech*(mLl + mLmq);
	mTd0_t = (mLlfd + mLmd) / (mRfd*mBase_OmMech);
	mXd_t = mOmMech*(mLl + mLmd - mLmd*mLmd / (mLlfd + mLmd));
	mEq_t = mVq + mXd_t*mId;
	mEf = mOmMech*(mLmd / mRfd) * mVfd;

	mVa = inverseParkTransform2(mThetaMech, mVd* mBase_V,  mVq* mBase_V,  mV0* mBase_v)(0);
	mVb = inverseParkTransform2(mThetaMech, mVd* mBase_V,  mVq* mBase_V,  mV0* mBase_v)(1);
	mVc = inverseParkTransform2(mThetaMech, mVd* mBase_V,  mVq* mBase_V,  mV0* mBase_v)(2);

	mIa = inverseParkTransform2(mThetaMech, mId* mBase_I,  mIq* mBase_I,  mI0* mBase_i)(0);
	mIb = inverseParkTransform2(mThetaMech, mId* mBase_I,  mIq* mBase_I,  mI0* mBase_i)(1);
	mIc = inverseParkTransform2(mThetaMech, mId* mBase_I,  mIq* mBase_I,  mI0* mBase_i)(2);

	va.setSourceValue(mVa);
	vb.setSourceValue(mVb);
	vc.setSourceValue(mVc);
}

// TODO: fix voltage sources
void EMT::Ph3::SynchronGeneratorDQSmplCompSource::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	//va.setVirtualNode(0, mVirtualNodes[0]);
	//vb.setVirtualNode(0, mVirtualNodes[1]);
	//vc.setVirtualNode(0, mVirtualNodes[2]);

	va.mnaApplySystemMatrixStamp(systemMatrix);
	vb.mnaApplySystemMatrixStamp(systemMatrix);
	vc.mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::SynchronGeneratorDQSmplCompSource::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	//va.setVirtualNode(0, mVirtualNodes[0]);
	//vb.setVirtualNode(0, mVirtualNodes[1]);
	//vc.setVirtualNode(0, mVirtualNodes[2]);

	va.mnaApplyRightSideVectorStamp(rightVector);
	vb.mnaApplyRightSideVectorStamp(rightVector);
	vc.mnaApplyRightSideVectorStamp(rightVector);
}

void EMT::Ph3::SynchronGeneratorDQSmplCompSource::mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time) {
	stepInPerUnit(mSystemOmega, mSystemTimeStep, time, mNumericalMethod);

	//va.setVirtualNode(0, mVirtualNodes[0]);
	//vb.setVirtualNode(0, mVirtualNodes[1]);
	//vc.setVirtualNode(0, mVirtualNodes[2]);

	va.mnaStep(systemMatrix, rightVector, leftVector, time);
	vb.mnaStep(systemMatrix, rightVector, leftVector, time);
	vc.mnaStep(systemMatrix, rightVector, leftVector, time);

	if (mLogLevel != Logger::Level::off) {
		Matrix logValues(fluxes().rows() + voltages().rows() + currents().rows() + 3, 1);
		logValues << fluxes(), voltages(), currents(), electricalTorque(), rotationalSpeed(), rotorPosition();
		mLog->LogNodeValues(time, logValues);
	}
}

void EMT::Ph3::SynchronGeneratorDQSmplCompSource::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{
	mIa = (1 / mBase_i) * mIa;
	mIb = (1 / mBase_i) * mIb;
	mIc = (1 / mBase_i) * mIc;

	// dq-transform of phase currents voltage
	mId = parkTransform2(mThetaMech, mIa, mIb, mIc)(0);
	mIq = parkTransform2(mThetaMech, mIa, mIb, mIc)(1);
	mI0 = parkTransform2(mThetaMech, mIa, mIb, mIc)(2);

	//Real A = -mRfd / (mLlfd + mLmd);
	//Real B = -mRfd*mLmd / (mLlfd + mLmd);
	//Real C = mVfd;
	//mPsifd = Trapezoidal(mPsifd, A, B, C, dt*mBase_OmMech, mId);

	mPsifd = mPsifd + dt*mBase_OmMech*(mVfd - mRfd*mIfd);
	mIfd = (mPsifd + mLmd*mId) / (mLlfd + mLmd);

	mPsid = -(mLmd + mLl)*mId + mLmd*mIfd;
	mPsiq = -(mLmq + mLl)*mIq;

	mVd = -mPsiq;
	mVq = mPsid;

	// Calculation of rotational speed with euler
	mElecTorque = (mPsid*mIq - mPsiq*mId);
	mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));
	// Calculation of rotor angular position
	mThetaMech = mThetaMech + dt * (mOmMech * mBase_OmMech);

	mVa = mBase_V  * inverseParkTransform2(mThetaMech, mVd, mVq, 0)(0);
	mVb = mBase_V  * inverseParkTransform2(mThetaMech, mVd, mVq, 0)(1);
	mVc = mBase_V  * inverseParkTransform2(mThetaMech, mVd, mVq, 0)(2);

	va.setSourceValue(mVa);
	vb.setSourceValue(mVb);
	vc.setSourceValue(mVc);

	mVoltages <<
		mVq,
		mVd,
		mVfd;

	mCurrents <<
		mIq,
		mId,
		mIfd;

	mFluxes <<
		mPsiq,
		mPsid,
		mPsifd;
}

void EMT::Ph3::SynchronGeneratorDQSmplCompSource::mnaPostStep(Matrix& rightVector, Matrix& leftVector, Real time) {
	if ( terminalNotGrounded(0) ) {
		mIa = va.current();
	}
	else {
		mIa = 0;
	}
	if ( terminalNotGrounded(1) ) {
		mIb = vb.current();
	}
	else {
		mIb = 0;
	}
	if ( simNode(2) >= 0) {
		mIc = vc.current();
	}
	else {
		mIc = 0;
	}
}

Matrix EMT::Ph3::SynchronGeneratorDQSmplCompSource::parkTransform2(Real theta, Real a, Real b, Real c)
{
	Matrix dq0vector(3, 1);

	// Park transform according to Kundur
	Real d, q;

	d = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	q = -2. / 3. * sin(theta)*a - 2. / 3. * sin(theta - 2. * M_PI / 3.)*b - 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	//Real zero;
	//zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector <<
		d,
		q,
		0;

	return dq0vector;
}

Matrix EMT::Ph3::SynchronGeneratorDQSmplCompSource::inverseParkTransform2(Real theta, Real d, Real q, Real zero)
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
