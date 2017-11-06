/** Simplified Synchron generator (EMT)
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
* @license GNU General Public License (version 3)
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

#include "SimplifiedSynchronGeneratorEMT.h"
#include "../IntegrationMethod.h"

using namespace DPsim;


SimplifiedSynchronGeneratorEMT::SimplifiedSynchronGeneratorEMT(String name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, bool logActive)
	: SynchGenBase(name, node1, node2, node3, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logActive) { }



SimplifiedSynchronGeneratorEMT::~SimplifiedSynchronGeneratorEMT() {
	if (mLogActive) {
		delete mLog;
	}
}


void SimplifiedSynchronGeneratorEMT::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt,
	Real initVoltAngle, Real initFieldVoltage, Real initMechPower)
{

	// Create matrices for state space representation
	if (mNumDampingWindings == 2)
	{
		mVoltages = Matrix::Zero(3, 1);
		mFluxes = Matrix::Zero(3, 1);
		mCurrents = Matrix::Zero(3, 1);
		mInductanceMat = Matrix::Zero(3, 3);
		mResistanceMat = Matrix::Zero(3, 3);
		mReactanceMat = Matrix::Zero(3, 3);
		mOmegaFluxMat = Matrix::Zero(3, 3);

		//Determinant of Lq(inductance matrix of q axis)
		//detLq = -mLmq*mLlkq2*(mLlkq1 + mLl) - mLl*mLlkq1*(mLlkq2 + mLmq);
		mInductanceMat <<
			-(mLl + mLmq), 0, 0,
			0, -(mLl + mLmd), mLmd,
			0, -mLmd, mLlfd + mLmd;
	

		//mResistanceMat <<
		//	mRs, 0, 0,
		//	0, mRs, 0,
		//	0, 0, -mRfd;

		mResistanceMat <<
			0, 0, 0,
			0, 0, 0,
			0, 0, -mRfd;


		mOmegaFluxMat <<
			0, 1, 0,
			-1, 0, 0,
			0, 0, 0;

	}

	// Determinant of Ld (inductance matrix of d axis)
	detLd = -(mLmd + mLl)*(mLlfd + mLmd) + mLmd*mLmd;

	mReactanceMat = mInductanceMat.inverse();

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

	mVa = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(0);
	mVb = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(1);
	mVc = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(2);

	mIa = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(0);
	mIb = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(1);
	mIc = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(2);
}


void SimplifiedSynchronGeneratorEMT::step(SystemModel& system, Real time) {

	stepInPerUnit(system.getOmega(), system.getTimeStep(), time, system.getNumMethod());

	// Update current source accordingly
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mIa);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, mIb);
	}
	if (mNode3 >= 0) {
		system.addRealToRightSideVector(mNode3, mIc);
	}

	if (mLogActive) {
		Matrix logValues(getFluxes().rows() + getVoltages().rows() + getCurrents().rows() + 3, 1);
		logValues << getFluxes(), getVoltages(), getCurrents(), getElectricalTorque(), getRotationalSpeed(), getRotorPosition();
		mLog->LogDataLine(time, logValues);
	}
}

void SimplifiedSynchronGeneratorEMT::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod) {

	mVa = (1 / mBase_v) * mVa;
	mVb = (1 / mBase_v) * mVb;
	mVc = (1 / mBase_v) * mVc;

	mIa = (1 / mBase_i) * mIa;
	mIb = (1 / mBase_i) * mIb;
	mIc = (1 / mBase_i) * mIc;

	// dq-transform of interface voltage
	mVd = parkTransform2(mThetaMech, mVa, mVb, mVc)(0);
	mVq = parkTransform2(mThetaMech, mVa, mVb, mVc)(1);
	mV0 = parkTransform2(mThetaMech, mVa, mVb, mVc)(2);


	Matrix A = (mResistanceMat*mReactanceMat - mOmMech*mOmegaFluxMat);
	Matrix B = Matrix::Identity(3, 3);

	Matrix Fluxes(3, 1);
	Fluxes(0, 0) = mPsiq;
	Fluxes(1, 0) = mPsid;
	Fluxes(2, 0) = mPsifd;

	Matrix dqVoltages(3, 1);
	dqVoltages(0, 0) = mVq;
	dqVoltages(1, 0) = mVd;
	dqVoltages(2, 0) = mVfd;

	Fluxes = Trapezoidal(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);


	mPsiq = Fluxes(0, 0);
	mPsid = Fluxes(1, 0);
	mPsifd = Fluxes(2, 0);


	////mOmMech = 1;

	//mPsid = mVq;
	//mPsiq = -mVd;
	//mPsifd = mPsifd + dt*mBase_OmMech*(mVfd - mRfd*mIfd);

	
	mId = ((mLlfd + mLmd) *mPsid - mLmd*mPsifd) / detLd;
	mIfd = (mLmd*mPsid - (mLl + mLmd)*mPsifd) / detLd;
	mIq = -mPsiq / (mLl + mLmq);

	mI0 = 0;



	// Calculation of rotational speed with euler
	mElecTorque = (mPsid*mIq - mPsiq*mId);
	mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));
	// Calculation of rotor angular position
	mThetaMech = mThetaMech + dt * (mOmMech * mBase_OmMech);

	mIa = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(0);
	mIb = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(1);
	mIc = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(2);

	mCurrents << mIq,
		mId,
		mIfd;

	mVoltages << mVq,
		mVd,
		mVfd;

	mFluxes << mPsiq,
		mPsid,
		mPsifd;

}

void SimplifiedSynchronGeneratorEMT::postStep(SystemModel& system) {
	if (mNode1 >= 0) {
		mVa = system.getRealFromLeftSideVector(mNode1);
	}
	else {
		mVa = 0;
	}
	if (mNode2 >= 0) {
		mVb = system.getRealFromLeftSideVector(mNode2);
	}
	else {
		mVb = 0;
	}
	if (mNode3 >= 0) {
		mVc = system.getRealFromLeftSideVector(mNode3);
	}
	else {
		mVc = 0;
	}
}

Matrix SimplifiedSynchronGeneratorEMT::parkTransform2(Real theta, Real a, Real b, Real c) {

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


Matrix SimplifiedSynchronGeneratorEMT::inverseParkTransform2(Real theta, Real d, Real q, Real zero) {

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
