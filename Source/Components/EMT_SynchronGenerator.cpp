/** Synchron generator (EMT)
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

#include "EMT_SynchronGenerator.h"
#include "../IntegrationMethod.h"

using namespace DPsim;

Component::EMT::SynchronGenerator::SynchronGenerator(String name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, bool logActive)
	: SynchronGeneratorBase(name, node1, node2, node3, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logActive) { }



Component::EMT::SynchronGenerator::~SynchronGenerator()
{
	if (mLogActive) {
		delete mLog;
	}
}


void Component::EMT::SynchronGenerator::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt,
	Real initVoltAngle, Real initFieldVoltage, Real initMechPower)
{

	// Create matrices for state space representation
	if (mNumDampingWindings == 2) {
		mVoltages = Matrix::Zero(7, 1);
		mFluxes = Matrix::Zero(7, 1);
		mCurrents = Matrix::Zero(7, 1);
		mInductanceMat = Matrix::Zero(7, 7);
		mResistanceMat = Matrix::Zero(7, 7);
		mReactanceMat = Matrix::Zero(7, 7);
		mOmegaFluxMat = Matrix::Zero(7, 7);

		//Determinant of Lq(inductance matrix of q axis)
		detLq = -mLmq*mLlkq2*(mLlkq1 + mLl) - mLl*mLlkq1*(mLlkq2 + mLmq);
		mInductanceMat <<
			-(mLl + mLmq), 0, 0, mLmq, mLmq, 0, 0,
			0, -(mLl + mLmd), 0, 0, 0, mLmd, mLmd,
			0, 0, -mLl, 0, 0, 0, 0,
			-mLmq, 0, 0, mLlkq1 + mLmq, mLmq, 0, 0,
			-mLmq, 0, 0, mLmq, mLlkq2 + mLmq, 0, 0,
			0, -mLmd, 0, 0, 0, mLlfd + mLmd, mLmd,
			0, -mLmd, 0, 0, 0, mLmd, mLlkd + mLmd;

		mResistanceMat <<
			mRs, 0, 0, 0, 0, 0, 0,
			0, mRs, 0, 0, 0, 0, 0,
			0, 0, mRs, 0, 0, 0, 0,
			0, 0, 0, -mRkq1, 0, 0, 0,
			0, 0, 0, 0, -mRkq2, 0, 0,
			0, 0, 0, 0, 0, -mRfd, 0,
			0, 0, 0, 0, 0, 0, -mRkd;

		mOmegaFluxMat <<
			0, 1, 0, 0, 0, 0, 0,
			-1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0;
	}
	else {
		mVoltages = Matrix::Zero(6, 1);
		mFluxes = Matrix::Zero(6, 1);
		mCurrents = Matrix::Zero(6, 1);
		mInductanceMat = Matrix::Zero(6, 6);
		mResistanceMat = Matrix::Zero(6, 6);
		mReactanceMat = Matrix::Zero(6, 6);
		mOmegaFluxMat = Matrix::Zero(6, 6);

		//Determinant of Lq(inductance matrix of q axis)
		detLq = -(mLl + mLmq)*(mLlkq1 + mLmq) + mLmq*mLmq;

		mInductanceMat <<
			-(mLl + mLmq), 0, 0, mLmq, 0, 0,
			0, -(mLl + mLmd), 0, 0, mLmd, mLmd,
			0, 0, -mLl, 0, 0, 0,
			-mLmq, 0, 0, mLlkq1 + mLmq, 0, 0,
			0, -mLmd, 0, 0, mLlfd + mLmd, mLmd,
			0, -mLmd, 0, 0, mLmd, mLlkd + mLmd;
		mResistanceMat <<
			mRs, 0, 0, 0, 0, 0,
			0, mRs, 0, 0, 0, 0,
			0, 0, mRs, 0, 0, 0,
			0, 0, 0, -mRkq1, 0, 0,
			0, 0, 0, 0, -mRfd, 0,
			0, 0, 0, 0, 0, -mRkd;

		mOmegaFluxMat <<
			0, 1, 0, 0, 0, 0,
			-1, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0;
	}
	// Determinant of Ld (inductance matrix of d axis)
	detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) + mLmd*mLmd*(mLlfd + mLlkd);

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


void Component::EMT::SynchronGenerator::step(SystemModel& system, Real time)
{
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

void Component::EMT::SynchronGenerator::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{
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

	if (numMethod == NumericalMethod::Trapezoidal_current) {

		if (mNumDampingWindings == 2)
		{
			Matrix A = (mReactanceMat*mResistanceMat);
			Matrix B = mReactanceMat;
			Matrix C = Matrix::Zero(7, 1);
			C(0, 0) = -mOmMech*mPsid;
			C(1, 0) = mOmMech*mPsiq;
			C = mReactanceMat*C;

			Matrix dqCurrents(7, 1);
			dqCurrents(0, 0) = mIq;
			dqCurrents(1, 0) = mId;
			dqCurrents(2, 0) = mI0;
			dqCurrents(3, 0) = mIkq1;
			dqCurrents(4, 0) = mIkq2;
			dqCurrents(5, 0) = mIfd;
			dqCurrents(6, 0) = mIkd;

			Matrix dqVoltages(7, 1);
			dqVoltages(0, 0) = mVq;
			dqVoltages(1, 0) = mVd;
			dqVoltages(2, 0) = mV0;
			dqVoltages(3, 0) = mVkq1;
			dqVoltages(4, 0) = mVkq2;
			dqVoltages(5, 0) = mVfd;
			dqVoltages(6, 0) = mVkd;
			dqCurrents = Trapezoidal(dqCurrents, A, B, C, dt*mOmMech, dqVoltages);

			mIq = dqCurrents(0, 0);
			mId = dqCurrents(1, 0);
			mI0 = dqCurrents(2, 0);
			mIkq1 = dqCurrents(3, 0);
			mIkq2 = dqCurrents(4, 0);
			mIfd = dqCurrents(5, 0);
			mIkd = dqCurrents(6, 0);

			//Calculation of currents based on inverse of inductance matrix
			mPsiq = -(mLl + mLmq)*mIq + mLmq*mIkq1 + mLmq*mIkq2;
			mPsid = -(mLl + mLmd)*mId + mLmd*mIfd + mLmd*mIkd;
			mPsi0 = -mLl*mI0;
			mPsikq1 = -mLmq*mIq + (mLlkq1 + mLmq)*mIkq1 + mLmq*mIkq2;
			mPsikq2 = -mLmq*mIq + mLmq*mIkq1 + (mLlkq2 + mLmq)*mIkq2;
			mPsifd = -mLmd*mId + (mLlfd + mLmd)*mIfd + mLmd*mIkd;
			mPsikd = -mLmd*mId + mLmd*mIfd + (mLlkd + mLmd)*mIkd;
		}
		else {
			Matrix A = (mReactanceMat*mResistanceMat);
			Matrix B = mReactanceMat;
			Matrix C = Matrix::Zero(6, 1);
			C(0, 0) = -mOmMech*mPsid;
			C(1, 0) = mOmMech*mPsiq;
			C = mReactanceMat*C;

			Matrix dqCurrents(6, 1);
			dqCurrents(0, 0) = mIq;
			dqCurrents(1, 0) = mId;
			dqCurrents(2, 0) = mI0;
			dqCurrents(3, 0) = mIkq1;
			dqCurrents(4, 0) = mIfd;
			dqCurrents(5, 0) = mIkd;

			Matrix dqVoltages(6, 1);
			dqVoltages(0, 0) = mVq;
			dqVoltages(1, 0) = mVd;
			dqVoltages(2, 0) = mV0;
			dqVoltages(3, 0) = mVkq1;
			dqVoltages(4, 0) = mVfd;
			dqVoltages(5, 0) = mVkd;

			dqCurrents = Trapezoidal(dqCurrents, A, B, C, dt*mBase_OmElec, dqVoltages);

			mIq = dqCurrents(0, 0);
			mId = dqCurrents(1, 0);
			mI0 = dqCurrents(2, 0);
			mIkq1 = dqCurrents(3, 0);
			mIfd = dqCurrents(4, 0);
			mIkd = dqCurrents(5, 0);

			//Calculation of currents based on inverse of inductance matrix
			mPsiq = -(mLl + mLmq)*mIq + mLmq*mIkq1;
			mPsid = -(mLl + mLmd)*mId + mLmd*mIfd + mLmd*mIkd;
			mPsi0 = -mLl*mI0;
			mPsikq1 = -mLmq*mIq + (mLlkq1 + mLmq)*mIkq1;
			mPsifd = -mLmd*mId + (mLlfd + mLmd)*mIfd + mLmd*mIkd;
			mPsikd = -mLmd*mId + mLmd*mIfd + (mLlkd + mLmd)*mIkd;
		}
	}
	else {

		// Calculation of rotational speed with euler
		mElecTorque = (mPsid*mIq - mPsiq*mId);
		mOmMech = mOmMech + dt * (1 / (2 * mH) * (mMechTorque - mElecTorque));

		//Calculation of flux
		if (mNumDampingWindings == 2) {
			Matrix A = (mResistanceMat*mReactanceMat - mOmMech*mOmegaFluxMat);
			Matrix B = Matrix::Identity(7, 7);

			Matrix Fluxes(7, 1);
			Fluxes(0, 0) = mPsiq;
			Fluxes(1, 0) = mPsid;
			Fluxes(2, 0) = mPsi0;
			Fluxes(3, 0) = mPsikq1;
			Fluxes(4, 0) = mPsikq2;
			Fluxes(5, 0) = mPsifd;
			Fluxes(6, 0) = mPsikd;

			Matrix dqVoltages(7, 1);
			dqVoltages(0, 0) = mVq;
			dqVoltages(1, 0) = mVd;
			dqVoltages(2, 0) = mV0;
			dqVoltages(3, 0) = mVkq1;
			dqVoltages(4, 0) = mVkq2;
			dqVoltages(5, 0) = mVfd;
			dqVoltages(6, 0) = mVkd;

			if (numMethod == NumericalMethod::Trapezoidal_flux)
				Fluxes = Trapezoidal(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);
			else
				Fluxes = Euler(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);

			mPsiq = Fluxes(0, 0);
			mPsid = Fluxes(1, 0);
			mPsi0 = Fluxes(2, 0);
			mPsikq1 = Fluxes(3, 0);
			mPsikq2 = Fluxes(4, 0);
			mPsifd = Fluxes(5, 0);
			mPsikd = Fluxes(6, 0);

		}
		else {
			Matrix A = (mResistanceMat*mReactanceMat - mOmMech*mOmegaFluxMat);
			Matrix B = Matrix::Identity(6, 6);

			Matrix Fluxes(6, 1);
			Fluxes(0, 0) = mPsiq;
			Fluxes(1, 0) = mPsid;
			Fluxes(2, 0) = mPsi0;
			Fluxes(3, 0) = mPsikq1;
			Fluxes(4, 0) = mPsifd;
			Fluxes(5, 0) = mPsikd;

			Matrix dqVoltages(6, 1);
			dqVoltages(0, 0) = mVq;
			dqVoltages(1, 0) = mVd;
			dqVoltages(2, 0) = mV0;
			dqVoltages(3, 0) = mVkq1;
			dqVoltages(4, 0) = mVfd;
			dqVoltages(5, 0) = mVkd;

			if (numMethod == NumericalMethod::Trapezoidal_flux)
				Fluxes = Trapezoidal(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);
			else
				Fluxes = Euler(Fluxes, A, B, dt*mBase_OmElec, dqVoltages);

			mPsiq = Fluxes(0, 0);
			mPsid = Fluxes(1, 0);
			mPsi0 = Fluxes(2, 0);
			mPsikq1 = Fluxes(3, 0);
			mPsifd = Fluxes(4, 0);
			mPsikd = Fluxes(5, 0);
		}


		// Calculation of currents based on inverse of inductance matrix
		mId = ((mLlfd*mLlkd + mLmd*(mLlfd + mLlkd))*mPsid - mLmd*mLlkd*mPsifd - mLlfd*mLmd*mPsikd) / detLd;
		mIfd = (mLlkd*mLmd*mPsid - (mLl*mLlkd + mLmd*(mLl + mLlkd))*mPsifd + mLmd*mLl*mPsikd) / detLd;
		mIkd = (mLmd*mLlfd*mPsid + mLmd*mLl*mPsifd - (mLmd*(mLlfd + mLl) + mLl*mLlfd)*mPsikd) / detLd;
		if (mNumDampingWindings == 2) {
			mIq = ((mLlkq1*mLlkq2 + mLmq*(mLlkq1 + mLlkq2))*mPsiq - mLmq*mLlkq2*mPsikq1 - mLmq*mLlkq1*mPsikq2) / detLq;
			mIkq1 = (mLmq*mLlkq2*mPsiq - (mLmq*(mLlkq2 + mLl) + mLl*mLlkq2)*mPsikq1 + mLmq*mLl*mPsikq2) / detLq;
			mIkq2 = (mLmq*mLlkq1*mPsiq + mLmq*mLl*mPsikq1 - (mLmq*(mLlkq1 + mLl) + mLl*mLlkq1)*mPsikq2) / detLq;
		}
		else {
			mIq = ((mLlkq1 + mLmq)*mPsiq - mLmq*mPsikq1) / detLq;
			mIkq1 = (mLmq*mPsiq - (mLl + mLmq)*mPsikq1) / detLq;
		}
		mI0 = -mPsi0 / mLl;
	}

	// Calculation of rotor angular position
	mThetaMech = mThetaMech + dt * (mOmMech * mBase_OmMech);

	mIa = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(0);
	mIb = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(1);
	mIc = mBase_i * inverseParkTransform2(mThetaMech, mId, mIq, mI0)(2);

	if (mNumDampingWindings == 2) {
		mCurrents << mIq,
			mId,
			mI0,
			mIkq1,
			mIkq2,
			mIfd,
			mIkd;

		mVoltages << mVq,
			mVd,
			mV0,
			mVkq1,
			mVkq2,
			mVfd,
			mVkd;

		mFluxes << mPsiq,
			mPsid,
			mPsi0,
			mPsikq1,
			mPsikq2,
			mPsifd,
			mPsikd;
	}
	else {
		mCurrents << mIq,
			mId,
			mI0,
			mIkq1,
			mIfd,
			mIkd;

		mVoltages << mVq,
			mVd,
			mV0,
			mVkq1,
			mVfd,
			mVkd;

		mFluxes << mPsiq,
			mPsid,
			mPsi0,
			mPsikq1,
			mPsifd,
			mPsikd;
	}
}

void Component::EMT::SynchronGenerator::postStep(SystemModel& system) {
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

Matrix Component::EMT::SynchronGenerator::parkTransform2(Real theta, Real a, Real b, Real c) {

	Matrix dq0vector(3, 1);

	// Park transform according to Kundur
	Real d, q;

	d =  2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	q = -2. / 3. * sin(theta)*a - 2. / 3. * sin(theta - 2. * M_PI / 3.)*b - 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	//Real zero;
	//zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector << d,
		q,
		0;

	return dq0vector;
}


Matrix Component::EMT::SynchronGenerator::inverseParkTransform2(Real theta, Real d, Real q, Real zero) {

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
