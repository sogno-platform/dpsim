/** Synchron generator
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

#include "Base_SynchronGenerator.h"

using namespace DPsim;

Components::SynchronGeneratorBase::SynchronGeneratorBase(String name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, LogLevel logLevel)
	: Base(name, node1, node2, node3, logLevel) {
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

	// Create logging file
	if (mLogLevel != LogLevel::NONE) {
		String filename = "SynGen_" + mName + ".csv";
		mLog = new Logger(filename);
	}

	if (Rkq2 == 0 && Llkq2 == 0) {
		mNumDampingWindings = 1;
	}
	else {
		mNumDampingWindings = 2;
	}

	initWithPerUnitParam(Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
}

void Components::SynchronGeneratorBase::initWithPerUnitParam(
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real H)
{
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

}

void Components::SynchronGeneratorBase::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
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
	Real init_psid = init_vq + mRs * init_iq;
	Real init_psiq = -init_vd - mRs * init_id;
	Real init_psifd = (mLmd + mLlfd) * init_ifd - mLmd * init_id;
	Real init_psid1 = mLmd * (init_ifd - init_id);
	Real init_psiq1 = -mLmq * init_iq;
	Real init_psiq2 = -mLmq * init_iq;

	// rotor mechanical variables
	Real init_Te = init_P + mRs * pow(init_it, 2.);

	// Initialize mechanical variables
	mOmMech = 1;
	mMechPower = initMechPower / mNomPower;
	mMechTorque = mMechPower / 1;
	mThetaMech = initVoltAngle + init_delta - PI / 2.;

	mVd = init_vd;
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
	mPsikd = init_psid1;
	mPsikq1 = init_psiq1;
	mPsikq2 = init_psiq2;
}

