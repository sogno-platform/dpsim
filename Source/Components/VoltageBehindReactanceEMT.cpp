/** Voltage behind reactance (EMT)
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

#include "VoltageBehindReactanceEMT.h"
#include "IntegrationMethod.h"

using namespace DPsim;

VoltageBehindReactanceEMT::VoltageBehindReactanceEMT(std::string name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia) {

	this->mNode1 = node1 - 1;
	this->mNode2 = node2 - 1;
	this->mNode3 = node3 - 1;

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

	// steady state per unit initial value
	initWithPerUnitParam(Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);

}
void VoltageBehindReactanceEMT::initWithPerUnitParam(
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

	if (mRkq2 == 0 && mLlkq2 == 0)
	{
		DampingWinding = 1;
	}

	//Dynamic mutual inductances
	mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);
	if (DampingWinding == 2)
	{
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1 + 1. / mLlkq2);

		A_flux <<
			-mRkq1 / mLlkq1*(1 - mDLmq / mLlkq1), mRkq1 / mLlkq1*(mDLmq / mLlkq2), 0, 0,
			mRkq2 / mLlkq2*(mDLmq / mLlkq1), -mRkq2 / mLlkq2*(1 - mDLmq / mLlkq2), 0, 0,
			0, 0, -mRfd / mLlfd*(1 - mDLmd / mLlfd), mRfd / mLlfd*(mDLmd / mLlkd),
			0, 0, mRkd / mLlkd*(mDLmd / mLlfd), -mRkd / mLlkd*(1 - mDLmd / mLlkd);
		B_flux <<
			mRkq1*mDLmq / mLlkq1, 0,
			mRkq2*mDLmq / mLlkq2, 0,
			0, mRfd*mDLmd / mLlfd,
			0, mRkd*mDLmd / mLlkd;
	}
		
	else
	{	
		mDLmq = 1. / (1. / mLmq + 1. / mLlkq1);

		A_flux = Matrix::Zero(3, 3);
		B_flux = Matrix::Zero(3, 2);
		C_flux = Matrix::Zero(3, 1);
		mRotorFlux = Matrix::Zero(3, 1);
		mDqStatorCurrents = Matrix::Zero(2, 1);
		mDqStatorCurrents_hist = Matrix::Zero(2, 1);

		A_flux <<
			-mRkq1 / mLlkq1*(1 - mDLmq / mLlkq1), 0, 0,
			0, -mRfd / mLlfd*(1 - mDLmd / mLlfd), mRfd / mLlfd*(mDLmd / mLlkd),
			0, mRkd / mLlkd*(mDLmd / mLlfd), -mRkd / mLlkd*(1 - mDLmd / mLlkd);
		B_flux <<
			mRkq1*mDLmq / mLlkq1, 0,
			0, mRfd*mDLmd / mLlfd,
			0, mRkd*mDLmd / mLlkd;		
	}
		

	mLa = (mDLmq + mDLmd) / 3.;
	mLb = (mDLmd - mDLmq) / 3.;

}


void VoltageBehindReactanceEMT::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle) {

	mResistanceMat <<
		mRs, 0, 0,
		0, mRs, 0,
		0, 0, mRs;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	mVa = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(0);
	mVb = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(1);
	mVc = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(2);

	mIa = inverseParkTransform(mThetaMech, mIq, mId, mI0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, mI0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, mI0)(2);
}

void VoltageBehindReactanceEMT::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle) {

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
	Real init_id = -init_it * sin(init_delta + init_pf);
	Real init_iq = -init_it * cos(init_delta + init_pf);

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
	mOmMech = 1;

	mIq = init_iq;
	mId = init_id;
	mI0 = 0;
	mIfd = init_ifd;
	mIkd = 0;
	mIkq1 = 0;
	mIkq2 = 0;

	mVd = init_vd;
	mVq = init_vq;
	mV0 = 0;
	mVfd = init_vfd;
	mVkd = 0;
	mVkq1 = 0;
	mVkq2 = 0;

	mPsiq = init_psiq;
	mPsid = init_psid;
	mPsi0 = 0;
	mPsifd = init_psifd;
	mPsikd = init_psid1;
	mPsikq1 = init_psiq1;
	mPsikq2 = init_psiq2;

	if (DampingWinding == 2) {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
	}
	else {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1);
	}
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);

	if (DampingWinding == 2) {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
			mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	}
	else {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) + (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq*mIq;
	}
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	// Initialize mechanical angle
	mThetaMech = initVoltAngle + init_delta;

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	//Initial inductance matrix
	mDInductanceMat <<
	mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
	-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
	-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);

	if (DampingWinding == 2) {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

}


void VoltageBehindReactanceEMT::step(SystemModel& system, Real fieldVoltage, Real mechPower, Real time) {

	stepInPerUnit(system.getOmega(), system.getTimeStep(), fieldVoltage, mechPower, time, system.getNumMethod());

	mVoltageVector = mVabc*mBase_v;
	mCurrentVector = mIabc*mBase_i;

}


void VoltageBehindReactanceEMT::stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower, Real time, NumericalMethod numMethod) {

	mVabc <<
		mVa,
		mVb,
		mVc;

	mIabc <<
		mIa,
		mIb,
		mIc;

	mDVabc <<
		mDVa,
		mDVb,
		mDVc;

	// Calculate mechanical variables with euler
	mMechPower = mechPower / mNomPower;
	mMechTorque = -(mMechPower / 1);
	mElecTorque = (mPsimd*mIq - mPsimq*mId);
	mOmMech = mOmMech + dt * (1. / (2. * mH) * (mElecTorque - mMechTorque));
	mThetaMech = mThetaMech + dt * (mOmMech* mBase_OmMech);

	// Calculate Inductance matrix and its derivative
	mDInductanceMat <<
		mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
		-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
		-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);
	pmDInductanceMat <<
		mLb*sin(2 * mThetaMech),  mLb*sin(2 * mThetaMech - 2 * PI / 3), mLb*sin(2 * mThetaMech + 2 * PI / 3),
		mLb*sin(2 * mThetaMech - 2 * PI / 3), mLb*sin(2 * mThetaMech - 4 * PI / 3), mLb*sin(2 * mThetaMech),
		mLb*sin(2 * mThetaMech + 2 * PI / 3), mLb*sin(2 * mThetaMech), mLb*sin(2 * mThetaMech + 4 * PI / 3);
	pmDInductanceMat = pmDInductanceMat * 2 * mOmMech;

	// Load resistance
	if (time < 0.1 || time > 0.2)
	{
		R_load <<
			1037.8378 / mBase_Z, 0, 0,
			0, 1037.8378 / mBase_Z, 0,
			0, 0, 1037.8378 / mBase_Z;
	}
	else
	{
		R_load <<
			0.001 / mBase_Z, 0, 0,
			0, 0.001 / mBase_Z, 0,
			0, 0, 0.001 / mBase_Z;
	}

	// Solve circuit - calculate stator currents and voltages
	if (numMethod == NumericalMethod::Trapezoidal_flux) 
		mIabc = Trapezoidal(mIabc, -mDInductanceMat.inverse()*(mResistanceMat + R_load + pmDInductanceMat), mDInductanceMat.inverse(), dt*mBase_OmElec, -mDVabc);
	else if (numMethod == NumericalMethod::Euler) 
		mIabc = Euler(mIabc, -mDInductanceMat.inverse()*(mResistanceMat + R_load + pmDInductanceMat), mDInductanceMat.inverse(), dt*mBase_OmElec, -mDVabc);
	mVabc = -R_load*mIabc;
	mIa = mIabc(0);
	mIb = mIabc(1);
	mIc = mIabc(2);
	mVa = mVabc(0);
	mVb = mVabc(1);
	mVc = mVabc(2);
	Real mIq_hist = mIq;
	Real mId_hist = mId;
	mIq = parkTransform(mThetaMech, mIa, mIb, mIc)(0);
	mId = parkTransform(mThetaMech, mIa, mIb, mIc)(1);
	mI0 = parkTransform(mThetaMech, mIa, mIb, mIc)(2);

	// Calculate rotor flux likanges
	if (DampingWinding == 2)
	{
		C_flux <<
			0,
			0,
			mVfd,
			0;

		mDqStatorCurrents <<
			mIq,
			mId;

		mDqStatorCurrents_hist <<
			mIq_hist,
			mId_hist;

		mRotorFlux <<
			mPsikq1,
			mPsikq2,
			mPsifd,
			mPsikd;

		if (numMethod == NumericalMethod::Trapezoidal_flux)
			mRotorFlux = Trapezoidal(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents, mDqStatorCurrents_hist);
		else if (numMethod == NumericalMethod::Euler)
			mRotorFlux = Euler(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents);

		mPsikq1 = mRotorFlux(0);
		mPsikq2 = mRotorFlux(1);
		mPsifd = mRotorFlux(2);
		mPsikd = mRotorFlux(3);

	}

	else
	{
		C_flux <<
			0,
			mVfd,
			0;

		mDqStatorCurrents <<
			mIq,
			mId;
		mDqStatorCurrents_hist <<
			mIq_hist,
			mId_hist;

		mRotorFlux <<
			mPsikq1,
			mPsifd,
			mPsikd;

		if (numMethod == NumericalMethod::Trapezoidal_flux)
			mRotorFlux = Trapezoidal(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents, mDqStatorCurrents_hist);
		else if (numMethod == NumericalMethod::Euler)
			mRotorFlux = Euler(mRotorFlux, A_flux, B_flux, C_flux, dt*mBase_OmElec, mDqStatorCurrents);

		mPsikq1 = mRotorFlux(0);
		mPsifd = mRotorFlux(1);
		mPsikd = mRotorFlux(2);
	}


	// Calculate dynamic flux likages
	if (DampingWinding == 2) {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1);
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);


	// Calculate dynamic voltages
	if (DampingWinding == 2) {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
			mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	}
	else {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) + (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq*mIq;
	}
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

}


void VoltageBehindReactanceEMT::postStep(SystemModel& system) {


}


Matrix VoltageBehindReactanceEMT::parkTransform(Real theta, Real a, Real b, Real c) {

	Matrix dq0vector(3, 1);

	Real q, d;

	q = 2. / 3. * cos(theta)*a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	d = 2. / 3. * sin(theta)*a + 2. / 3. * sin(theta - 2. * M_PI / 3.)*b + 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;

	//Real zero;
	//zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector << q,
		d,
		0;

	return dq0vector;
}



Matrix VoltageBehindReactanceEMT::inverseParkTransform(Real theta, Real q, Real d, Real zero) {

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

