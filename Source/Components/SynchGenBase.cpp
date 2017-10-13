/** Synchron generator
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

#include "SynchGenBase.h"

using namespace DPsim;

SynchGenBase::SynchGenBase(std::string name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, bool logActive)
	: BaseComponent(name, node1, node2, node3, logActive) {
	
	mNomPower = nomPower;
	mNomVolt = nomVolt;
	mNomFreq = nomFreq;
	mPoleNumber = poleNumber;
	mNomFieldCur = nomFieldCur;

	// base stator values
	mBase_V_RMS = mNomVolt / sqrt(3);
	mBase_v = mBase_V_RMS * sqrt(2);
	mBase_I_RMS = mNomPower / (3 * mBase_V_RMS);
	mBase_i = mBase_I_RMS * sqrt(2);
	mBase_Z = mBase_v / mBase_i;
	mBase_OmElec = 2 * DPS_PI * mNomFreq;
	mBase_OmMech = mBase_OmElec / (mPoleNumber / 2);
	mBase_L = mBase_Z / mBase_OmElec;
	mBase_Psi = mBase_L * mBase_i;
	mBase_T = mNomPower / mBase_OmMech;
	
	if (Rkq2 == 0 && Llkq2 == 0) {
		mNumDampingWindings = 1;
	}
	else {
		mNumDampingWindings = 2;
	}

	initWithPerUnitParam(Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
}

void SynchGenBase::initWithPerUnitParam(
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real H) {	

	// base rotor values
	mBase_ifd = Lmd * mNomFieldCur;
	mBase_vfd = mNomPower / mBase_ifd;
	mBase_Zfd = mBase_vfd / mBase_ifd;
	mBase_Lfd = mBase_Zfd / mBase_OmElec;

	mRs = Rs;
	mLl = Ll;
	mLmd = Lmd;
	mLmd0 = Lmd0;
	mLmq = Lmq;
	mLmq0 = Lmq0;
	mRfd = Rfd;
	mLlfd = Llfd;
	mRkd = Rkd;
	mLlkd = Llkd;
	mRkq1 = Rkq1;
	mLlkq1 = Llkq1;
	mRkq2 = Rkq2;
	mLlkq2 = Llkq2;
	mH = H;
	// Additional inductances according to Krause
	mLaq = 1 / (1 / mLmq + 1 / mLl + 1 / mLlkq1 + 1 / mLlkq2);
	mLad = 1 / (1 / mLmd + 1 / mLl + 1 / mLlkd + 1 / mLlfd);

	// Determinant of Ld (inductance matrix of d axis)
	detLd = (mLmd + mLl)*(-mLlfd*mLlkd - mLlfd*mLmd - mLmd*mLlkd) + mLmd*mLmd*(mLlfd + mLlkd);
	
	// Determinant of Lq (inductance matrix of q axis)
	if (mNumDampingWindings == 2) {
		detLq = -mLmq*mLlkq2*(mLlkq1 + mLl) - mLl*mLlkq1*(mLlkq2 + mLmq);
	}
	else {
		detLq = -(mLl + mLmq)*(mLlkq1 + mLmq) + mLmq*mLmq;
	}
}

void SynchGenBase::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt,
	Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	// Create matrices for state space representation
	if (mNumDampingWindings == 2) {
		mInductanceMat = Matrix::Zero(7, 7);
		mResistanceMat = Matrix::Zero(7, 7);
		mReactanceMat = Matrix::Zero(7, 7);
		mOmegaFluxMat = Matrix::Zero(7, 7);

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
		mInductanceMat = Matrix::Zero(6, 6);
		mResistanceMat = Matrix::Zero(6, 6);
		mReactanceMat = Matrix::Zero(6, 6);
		mOmegaFluxMat = Matrix::Zero(6, 6);

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

	mReactanceMat = mInductanceMat.inverse();

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);
}

void SynchGenBase::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	double init_P = initActivePower / mNomPower;
	double init_Q = initReactivePower / mNomPower;
	double init_S = sqrt(pow(init_P, 2.) + pow(init_Q, 2.));
	double init_vt = initTerminalVolt / mBase_v;
	double init_it = init_S / init_vt;

	// power factor
	double init_pf = acos(init_P / init_S);

	// load angle
	double init_delta = atan(((mLmq + mLl) * init_it * cos(init_pf) - mRs * init_it * sin(init_pf)) /
		(init_vt + mRs * init_it * cos(init_pf) + (mLmq + mLl) * init_it * sin(init_pf)));
	double init_delta_deg = init_delta / DPS_PI * 180;

	// dq stator voltages and currents
	double init_vd = init_vt * sin(init_delta);
	double init_vq = init_vt * cos(init_delta);
	double init_id = init_it * sin(init_delta + init_pf);
	double init_iq = init_it * cos(init_delta + init_pf);

	// rotor voltage and current
	double init_ifd = (init_vq + mRs * init_iq + (mLmd + mLl) * init_id) / mLmd;
	double init_vfd = mRfd * init_ifd;

	// flux linkages
	double init_psid = init_vq + mRs * init_iq;
	double init_psiq = -init_vd - mRs * init_id;
	double init_psifd = (mLmd + mLlfd) * init_ifd - mLmd * init_id;
	double init_psid1 = mLmd * (init_ifd - init_id);
	double init_psiq1 = -mLmq * init_iq;
	double init_psiq2 = -mLmq * init_iq;

	// rotor mechanical variables
	double init_Te = init_P + mRs * pow(init_it, 2.);
			
	// Initialize mechanical variables
	mOmMech = 1;
	mMechPower = initMechPower / mNomPower;
	mMechTorque = mMechPower / 1;
	mThetaMech = initVoltAngle + init_delta - PI / 2.;
}

