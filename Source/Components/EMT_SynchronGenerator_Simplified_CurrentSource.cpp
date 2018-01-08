/** Simplified Synchron generator (EMT)
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
*
* DPsim
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

#include "EMT_SynchronGenerator_Simplified_CurrentSource.h"
#include "../IntegrationMethod.h"

using namespace DPsim;

Component::EMT::SynchronGeneratorSimplifiedCurrentSource::SynchronGeneratorSimplifiedCurrentSource(String name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, bool logActive)
	: SynchronGeneratorBase(name, node1, node2, node3, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logActive),
		va("va", 0, 1, mVa),
		vb("vb", 0, 2, mVb),
		vc("vc", 0, 3, mVc)
{
	mNumVirtualNodes = 3;
	mVirtualNodes = { 0, 0, 0 };
}

Component::EMT::SynchronGeneratorSimplifiedCurrentSource::~SynchronGeneratorSimplifiedCurrentSource()
{
	if (mLogActive) {
		delete mLog;
	}
}

void Component::EMT::SynchronGeneratorSimplifiedCurrentSource::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt,
	Real initVoltAngle, Real initFieldVoltage, Real initMechPower)
{
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

	mVa = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(0);
	mVb = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(1);
	mVc = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(2);

	mIa = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(0);
	mIb = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(1);
	mIc = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(2);

	va.setVoltage(mVa);
	vb.setVoltage(mVb);
	vc.setVoltage(mVc);
}

void Component::EMT::SynchronGeneratorSimplifiedCurrentSource::applySystemMatrixStamp(SystemModel& system)
{
	va.setVirtualNode(0, mVirtualNodes[0]);
	vb.setVirtualNode(0, mVirtualNodes[1]);
	vc.setVirtualNode(0, mVirtualNodes[2]);

	va.applySystemMatrixStamp(system);
	vb.applySystemMatrixStamp(system);
	vc.applySystemMatrixStamp(system);
}

void Component::EMT::SynchronGeneratorSimplifiedCurrentSource::applyRightSideVectorStamp(SystemModel& system)
{
	va.setVirtualNode(0, mVirtualNodes[0]);
	vb.setVirtualNode(0, mVirtualNodes[1]);
	vc.setVirtualNode(0, mVirtualNodes[2]);

	va.applyRightSideVectorStamp(system);
	vb.applyRightSideVectorStamp(system);
	vc.applyRightSideVectorStamp(system);
}

void Component::EMT::SynchronGeneratorSimplifiedCurrentSource::step(SystemModel& system, Real time)
{
	stepInPerUnit(system.getOmega(), system.getTimeStep(), time, system.getNumMethod());

	va.setVirtualNode(0, mVirtualNodes[0]);
	vb.setVirtualNode(0, mVirtualNodes[1]);
	vc.setVirtualNode(0, mVirtualNodes[2]);

	va.step(system, time);
	vb.step(system, time);
	vc.step(system, time);

	if (mLogActive) {
		Matrix logValues(getFluxes().rows() + getVoltages().rows() + getCurrents().rows() + 3, 1);
		logValues << getFluxes(), getVoltages(), getCurrents(), getElectricalTorque(), getRotationalSpeed(), getRotorPosition();
		mLog->LogDataLine(time, logValues);
	}
}

void Component::EMT::SynchronGeneratorSimplifiedCurrentSource::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
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

	mVa = mBase_v * inverseParkTransform2(mThetaMech, mVd, mVq, 0)(0);
	mVb = mBase_v * inverseParkTransform2(mThetaMech, mVd, mVq, 0)(1);
	mVc = mBase_v * inverseParkTransform2(mThetaMech, mVd, mVq, 0)(2);

	va.setVoltage(mVa);
	vb.setVoltage(mVb);
	vc.setVoltage(mVc);

	mVoltages << mVq,
		mVd,
		mVfd;

	mCurrents << mIq,
		mId,
		mIfd;

	mFluxes << mPsiq,
		mPsid,
		mPsifd;
}

void Component::EMT::SynchronGeneratorSimplifiedCurrentSource::postStep(SystemModel& system)
{
	if (mNode1 >= 0) {
		mIa = va.getCurrent(system).real();
	}
	else {
		mIa = 0;
	}
	if (mNode2 >= 0) {
		mIb = vb.getCurrent(system).real();
	}
	else {
		mIb = 0;
	}
	if (mNode3 >= 0) {
		mIc = vc.getCurrent(system).real();
	}
	else {
		mIc = 0;
	}
}

Matrix Component::EMT::SynchronGeneratorSimplifiedCurrentSource::parkTransform2(Real theta, Real a, Real b, Real c)
{
	Matrix dq0vector(3, 1);

	// Park transform according to Kundur
	Real d, q;

	d = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	q = -2. / 3. * sin(theta)*a - 2. / 3. * sin(theta - 2. * M_PI / 3.)*b - 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	//Real zero;
	//zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector << d,
		q,
		0;

	return dq0vector;
}

Matrix Component::EMT::SynchronGeneratorSimplifiedCurrentSource::inverseParkTransform2(Real theta, Real d, Real q, Real zero)
{
	Matrix abcVector(3, 1);

	// Park transform according to Kundur
	Real a, b, c;

	a = cos(theta)*d - sin(theta)*q + 1.*zero;
	b = cos(theta - 2. * M_PI / 3.)*d - sin(theta - 2. * M_PI / 3.)*q + 1.*zero;
	c = cos(theta + 2. * M_PI / 3.)*d - sin(theta + 2. * M_PI / 3.)*q + 1.*zero;

	abcVector << a,
		b,
		c;

	return abcVector;
}
