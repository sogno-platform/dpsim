/** Voltage behind reactance (DP)
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

#include "DP_SynchronGenerator_VBR.h"
#include "../IntegrationMethod.h"

using namespace DPsim;

Components::DP::SynchronGeneratorVBR::SynchronGeneratorVBR(String name, Int node1, Int node2, Int node3,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, Logger::Level logLevel)
	: SynchronGeneratorBase(name, node1, node2, node3, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logLevel)
{
}

Components::DP::SynchronGeneratorVBR::~SynchronGeneratorVBR() {

}

void Components::DP::SynchronGeneratorVBR::addExciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd)
{
	mExciter = Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lad, Rfd);
	mExciter.initialize(1, 1);

	mHasExciter = true;
}

void Components::DP::SynchronGeneratorVBR::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef)
{
	mTurbineGovernor = TurbineGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
	mTurbineGovernor.initialize(PmRef, Tm_init);
	mHasTurbineGovernor = true;
}

void Components::DP::SynchronGeneratorVBR::initialize(Real om, Real dt,
	Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower)
{
	mResistanceMat = Matrix::Zero(3, 3);
	mResistanceMat <<
		mRs, 0, 0,
		0, mRs, 0,
		0, 0, mRs;

	R_load <<
		1037.8378 / mBase_Z, 0, 0, 0, 0, 0,
		0, 1037.8378 / mBase_Z, 0, 0, 0, 0,
		0, 0, 1037.8378 / mBase_Z, 0, 0, 0,
		0, 0, 0, 1037.8378 / mBase_Z, 0, 0,
		0, 0, 0, 0, 1037.8378 / mBase_Z, 0,
		0, 0, 0, 0, 0, 1037.8378 / mBase_Z;

	//Dynamic mutual inductances
	mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);
	if (mNumDampingWindings == 2) {
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
	else {
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

	LD0 <<
		(mLl + mLa), -mLa / 2, -mLa / 2,
		-mLa / 2, mLl + mLa, -mLa / 2,
		-mLa / 2, -mLa / 2, mLl + mLa;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

	mThetaMech2 = mThetaMech + PI/2;
	mTheta0 = mThetaMech2;
	mMechTorque = -mMechTorque;
	mIq = -mIq;
	mId = -mId;

	// VBR Model Dynamic variables
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
	if (mNumDampingWindings == 2) {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
			mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	}
	else {
		mDPsiq = mDLmq*(mPsikq1 / mLlkq1);
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) + (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq*mIq;
		mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
	}
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	mDVaRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(0);
	mDVbRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(1);
	mDVcRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(2);
	mDVaIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(3);
	mDVbIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(4);
	mDVcIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(5);
	mDVabc <<
		mDVaRe,
		mDVbRe,
		mDVcRe,
		mDVaIm,
		mDVbIm,
		mDVcIm;
	mDVabc_hist = mDVabc;

	mVaRe = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0)(0);
	mVbRe = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0)(1);
	mVcRe = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0)(2);
	mVaIm = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0)(3);
	mVbIm = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0)(4);
	mVcIm = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0)(5);

	mIaRe = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(0);
	mIbRe = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(1);
	mIcRe = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(2);
	mIaIm = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(3);
	mIbIm = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(4);
	mIcIm = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0)(5);

}

void Components::DP::SynchronGeneratorVBR::step(SystemModel& system, Real time)
{
	stepInPerUnit(system.getOmega(), system.getTimeStep(), time, system.getNumMethod());

	R_load = system.getCurrentSystemMatrix().inverse() / mBase_Z;

	// Update current source accordingly
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, Complex(-mIaRe*mBase_i, -mIaIm*mBase_i));
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, Complex(-mIbRe*mBase_i, -mIbIm*mBase_i));
	}
	if (mNode3 >= 0) {
		system.addCompToRightSideVector(mNode3, Complex(-mIcRe*mBase_i, -mIcIm*mBase_i));
	}

	if (mLogLevel != Logger::Level::NONE) {
		Matrix logValues(getRotorFluxes().rows() + getDqStatorCurrents().rows() + 3, 1);
		logValues << getRotorFluxes(), getDqStatorCurrents(), getElectricalTorque(), getRotationalSpeed(), getRotorPosition();
		mLog->LogDataLine(time, logValues);
	}

}

void Components::DP::SynchronGeneratorVBR::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{
	mIabc <<
		mIaRe,
		mIbRe,
		mIcRe,
		mIaIm,
		mIbIm,
		mIcIm;

	// Calculate mechanical variables with euler
	if (mHasTurbineGovernor == true) {
		mMechTorque = -mTurbineGovernor.step(mOmMech, 1, 0.001, dt);
	}

	mElecTorque = (mPsimd*mIq - mPsimq*mId);
	mOmMech = mOmMech + dt * (1. / (2. * mH) * (mElecTorque - mMechTorque));
	mThetaMech = mThetaMech + dt * ((mOmMech - 1) * mBase_OmMech);
	mThetaMech2 = mThetaMech2 + dt * (mOmMech* mBase_OmMech);

	// Calculate equivalent Resistance and inductance
	CalculateLandR(time);

	// Solve circuit - calculate stator currents
	if (numMethod == NumericalMethod::Trapezoidal_flux)
		mIabc = Trapezoidal(mIabc, -L_EQ.inverse()*(R_EQ + R_load), L_EQ.inverse(), dt*mBase_OmElec, -mDVabc, -mDVabc_hist);
	else
		mIabc = Euler(mIabc, -L_EQ.inverse()*(R_EQ + R_load), L_EQ.inverse(), dt*mBase_OmElec, -mDVabc);

	mIaRe = mIabc(0);
	mIbRe = mIabc(1);
	mIcRe = mIabc(2);
	mIaIm = mIabc(3);
	mIbIm = mIabc(4);
	mIcIm = mIabc(5);
	Real mIq_hist = mIq;
	Real mId_hist = mId;
	mIq = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(0);
	mId = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(1);
	mI0 = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(2);

	if (mHasExciter == true) {
		// dq-transform of interface voltage
		mVd = R_load(0, 0)*mId;
		mVq = R_load(0, 0)*mIq;
		mVfd = mExciter.step(mVd, mVq, 1, dt);
	}

	// Calculate rotor flux likanges
	if (mNumDampingWindings == 2) {
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
	else {
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
	if (mNumDampingWindings == 2) {
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
	if (mNumDampingWindings == 2) {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
			mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	}
	else {
		mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) + (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq*mIq;
	}
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	mDVaRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(0);
	mDVbRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(1);
	mDVcRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(2);
	mDVaIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(3);
	mDVbIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(4);
	mDVcIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(5);
	mDVabc_hist = mDVabc;
	mDVabc <<
		mDVaRe,
		mDVbRe,
		mDVcRe,
		mDVaIm,
		mDVbIm,
		mDVcIm;

#if 0
	// Load resistance
	if (time < 0.1 || time > 0.2) {
		R_load <<
			1037.8378 / mBase_Z, 0, 0, 0, 0, 0,
			0, 1037.8378 / mBase_Z, 0, 0, 0, 0,
			0, 0, 1037.8378 / mBase_Z, 0, 0, 0,
			0, 0, 0, 1037.8378 / mBase_Z, 0, 0,
			0, 0, 0, 0, 1037.8378 / mBase_Z, 0,
			0, 0, 0, 0, 0, 1037.8378 / mBase_Z;
	}
	else {
		R_load <<
			0.001 / mBase_Z, 0, 0, 0, 0, 0,
			0, 0.001 / mBase_Z, 0, 0, 0, 0,
			0, 0, 0.001 / mBase_Z, 0, 0, 0,
			0, 0, 0, 0.001 / mBase_Z, 0, 0,
			0, 0, 0, 0, 0.001 / mBase_Z, 0,
			0, 0, 0, 0, 0, 0.001 / mBase_Z;
	}
#endif
}

void Components::DP::SynchronGeneratorVBR::CalculateLandR(Real time)
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

	//Real b_Re2 = cos(2*mThetaMech2);
	//Real b_Im2 = sin(2*mThetaMech2);
	Real b_Re = cos(2 * mOmMech* mBase_OmMech*time);
	Real b_Im = sin(2 * mOmMech* mBase_OmMech*time);
	Real c_Re = cos(2 * mThetaMech2 - 2 * 1*mBase_OmMech*time - 2 * mTheta0);
	Real c_Im = sin(2 * mThetaMech2 - 2 * 1*mBase_OmMech*time - 2 * mTheta0);

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

	Re_R = mResistanceMat + (2 * mOmMech + 1) / 2. * (L1_Re*b_Im + L1_Im*b_Re);
	Im_R = LD0 - (2 * mOmMech + 1) / 2. *(L1_Re*b_Re - L1_Im*b_Im);
	Re_L = LD0 - 1. / 2.*(L1_Re*b_Re - L1_Im*b_Im);
	Im_L = -1. / 2.*(L1_Re*b_Im + L1_Im*b_Re);
	Re_R2 = 1. / 2.*(2 * mOmMech - 1)*(L1_Im*c_Re + L1_Re*c_Im);
	Im_R2 = -1. / 2.*(2 * mOmMech - 1)*(L1_Re*c_Re - L1_Im*c_Im);
	Re_L2 = -1. / 2.*(L1_Re*c_Re - L1_Im*c_Im);
	Im_L2 = -1. / 2.*(L1_Im*c_Re + L1_Re*c_Im);

	R_EQ <<
		Re_R + Re_R2, -Im_R + Im_R2,
		Im_R + Im_R2, Re_R - Re_R2;
	L_EQ <<
		Re_L + Re_L2, -Im_L + Im_L2,
		Im_L + Im_L2, Re_L - Re_L2;

}

void Components::DP::SynchronGeneratorVBR::postStep(SystemModel& system)
{
}

Matrix Components::DP::SynchronGeneratorVBR::abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm)
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
		pnzVector(1, 0).imag(),
		pnzVector(1, 0).real(),
		0;

	return dq0Vector;
}

Matrix Components::DP::SynchronGeneratorVBR::dq0ToAbcTransform(Real theta, Real d, Real q, Real zero)
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
		Complex(d, q),
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
