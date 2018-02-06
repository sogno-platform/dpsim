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

#include "EMT_SynchronGenerator_Simplified.h"
#include "../IntegrationMethod.h"

using namespace DPsim;

Components::EMT::SynchronGeneratorSimplified::SynchronGeneratorSimplified(String name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, Logger::Level logLevel)
	: SynchronGeneratorBase(name, node1, node2, node3, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logLevel) {
}

Components::EMT::SynchronGeneratorSimplified::~SynchronGeneratorSimplified()
{
	if (mLogLevel != Logger::Level::NONE) {
		delete mLog;
	}
}

void Components::EMT::SynchronGeneratorSimplified::initialize(Real om, Real dt,
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

	mVd = -mPsiq;
	mVq = mPsid;

	mVa = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(0);
	mVb = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(1);
	mVc = inverseParkTransform2(mThetaMech, mVd* mBase_v, mVq* mBase_v, mV0* mBase_v)(2);

	mIa = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(0);
	mIb = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(1);
	mIc = inverseParkTransform2(mThetaMech, mId* mBase_i, mIq* mBase_i, mI0* mBase_i)(2);
}

void Components::EMT::SynchronGeneratorSimplified::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
		Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower)
{
		// Electrical variables
		Real init_P = initActivePower / mNomPower;
		Real init_Q = initReactivePower / mNomPower;
		Real init_S = sqrt(pow(init_P, 2.) + pow(init_Q, 2.));
		Real init_vt = initTerminalVolt / mBase_v;
		Real init_it = init_S / init_vt;

		// power factor
		Real init_pf = acos(init_P / init_S);

		// load angle
		Real init_delta = atan(((mLmq + mLl) * init_it * cos(init_pf) - mRs * init_it * sin(init_pf)) /
				(init_vt + mRs * init_it * cos(init_pf) + (mLmq + mLl) * init_it * sin(init_pf)));
		Real init_delta_deg = init_delta / DPS_PI * 180;

		// dq stator voltages and currents
		Real init_vd = init_vt * sin(init_delta);
		Real init_vq = init_vt * cos(init_delta);
		Real init_id = init_it * sin(init_delta + init_pf);
		Real init_iq = init_it * cos(init_delta + init_pf);

		// rotor voltage and current
		Real init_ifd = (init_vq + mRs * init_iq + (mLmd + mLl) * init_id) / mLmd;
		Real init_vfd = mRfd * init_ifd;

		// flux linkages
		Real init_psid = -(mLl + mLmd)*init_id + mLmd*init_ifd;
		Real init_psiq = -(mLl + mLmq)*init_iq;
		Real init_psifd = (mLmd + mLlfd) * init_ifd - mLmd * init_id;

		// rotor mechanical variables
		Real init_Te = init_P + mRs * pow(init_it, 2.);

		// Initialize mechanical variables
		mOmMech = 1;
		mMechPower = initMechPower / mNomPower;
		mMechTorque = mMechPower / 1;

		mThetaMech = initVoltAngle + init_delta - PI / 2.;

		mVd = -init_psiq;
		mVq = init_vq;
		mV0 = 0;
		mVfd = init_vfd;
		mVkd = 0;
		mVkq1 = 0;
		mVkq2 = 0;

		mIq = init_iq;
		mId = init_id;
		mI0 = 0;
		mIfd = init_ifd;
		mIkd = 0;
		mIkq1 = 0;
		mIkq2 = 0;

		mPsiq = init_psiq;
		mPsid = init_psid;
		mPsi0 = 0;
		mPsifd = init_psifd;

}

void Components::EMT::SynchronGeneratorSimplified::step(SystemModel& system, Real time)
{

	mR_load = system.getCurrentSystemMatrix().inverse();
	mR_load = mR_load / mBase_Z;

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

	if (mLogLevel != Logger::Level::NONE) {
		Matrix logValues(getFluxes().rows() + getVoltages().rows() + getCurrents().rows() + 3, 1);
		logValues << getFluxes(), getVoltages(), getCurrents(), getElectricalTorque(), getRotationalSpeed(), getRotorPosition();
		mLog->LogDataLine(time, logValues);
	}
}

void Components::EMT::SynchronGeneratorSimplified::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{


	// Calculation of rotational speed with euler
	mElecTorque = (mPsid*mIq - mPsiq*mId);
	mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));
	// Calculation of rotor angular position
	mThetaMech = mThetaMech + dt * (mOmMech * mBase_OmMech);


	// Using fundamental parameters

	mPsifd = mPsifd + dt*mBase_OmMech*(mVfd - mRfd*mIfd);

	Matrix mR_eq = Matrix::Zero(2, 2);
	Matrix mE_eq = Matrix::Zero(2, 1);
	Matrix mIdq = Matrix::Zero(2, 1);
	mR_eq <<
			-(mRs + mR_load(0, 0)), (mLl + mLmq),
			-(mLl + mLmd) + mLmd*mLmd / (mLlfd + mLmd), -(mRs + mR_load(0, 0));
	mE_eq <<
			0,
			(mLmd)*mPsifd / (mLlfd + mLmd);

	mIdq = -mR_eq.inverse()*mE_eq;

	mId = mIdq(0);
	mIq = mIdq(1);
	mIfd = (mPsifd + mLmd*mId) / (mLlfd + mLmd);

	
	Matrix Fluxes(3, 1);

	Matrix Currents(3, 1);
	Currents(0, 0) = mIq;
	Currents(1, 0) = mId;
	Currents(2, 0) = mIfd;

	Fluxes = mInductanceMat*Currents;

	mPsiq = Fluxes(0, 0);
	mPsid = Fluxes(1, 0);
	mPsifd = Fluxes(2, 0);

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

void Components::EMT::SynchronGeneratorSimplified::postStep(SystemModel& system)
{
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

Matrix Components::EMT::SynchronGeneratorSimplified::parkTransform2(Real theta, Real a, Real b, Real c)
{
	Matrix dq0vector(3, 1);

	// Park transform according to Kundur
	Real d, q, zero;

	d = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	q = -2. / 3. * sin(theta)*a - 2. / 3. * sin(theta - 2. * M_PI / 3.)*b - 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	//Real zero;
	zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector << d,
		q,
		0;

	return dq0vector;
}

Matrix Components::EMT::SynchronGeneratorSimplified::inverseParkTransform2(Real theta, Real d, Real q, Real zero)
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
