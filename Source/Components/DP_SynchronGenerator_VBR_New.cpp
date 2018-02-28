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

#include "DP_SynchronGenerator_VBR_New.h"
#include "../IntegrationMethod.h"
#include <chrono>

using namespace DPsim;

Components::DP::SynchronGeneratorVBRNew::SynchronGeneratorVBRNew(String name, Int node1, Int node2, Int node3,
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

Components::DP::SynchronGeneratorVBRNew::~SynchronGeneratorVBRNew() {

}

void Components::DP::SynchronGeneratorVBRNew::addExciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd)
{
		mExciter = Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lad, Rfd);
		mExciter.initialize(1, 1);

		mHasExciter = true;
}

void Components::DP::SynchronGeneratorVBRNew::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef)
{
		mTurbineGovernor = TurbineGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
		mTurbineGovernor.initialize(PmRef, Tm_init);
		mHasTurbineGovernor = true;
}

void Components::DP::SynchronGeneratorVBRNew::initialize(Real om, Real dt,
		Real initActivePower, Real initReactivePower,
		Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower)
{
		mResistanceMat = Matrix::Zero(3, 3);
		mResistanceMat <<
				mRs, 0, 0,
				0, mRs, 0,
				0, 0, mRs;

		//Dynamic mutual inductances
		mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);

		if (mNumDampingWindings == 2)
				mDLmq = 1. / (1. / mLmq + 1. / mLlkq1 + 1. / mLlkq2);
		else
		{
				mDLmq = 1. / (1. / mLmq + 1. / mLlkq1);
				K1a = Matrix::Zero(2, 1);
				K1 = Matrix::Zero(2, 1);
		}

		mLa = (mDLmq + mDLmd) / 3.;
		mLb = (mDLmd - mDLmq) / 3.;

		LD0 <<
				(mLl + mLa), -mLa / 2, -mLa / 2,
				-mLa / 2, mLl + mLa, -mLa / 2,
				-mLa / 2, -mLa / 2, mLl + mLa;

		// steady state per unit initial value
		initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initFieldVoltage, initMechPower);

		mThetaMech2 = mThetaMech + PI / 2;
		mTheta0 = mThetaMech2;
		mMechTorque = -mMechTorque;
		mIq = -mIq;
		mId = -mId;

		CalculateAuxiliarConstants(dt*mBase_OmElec);

		if (mNumDampingWindings == 2)
				mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
		else
				mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);

		mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

		mDqStatorCurrents <<
				mIq,
				mId;

		mPsikq1kq2 <<
				mPsikq1,
				mPsikq2;
		mPsifdkd <<
				mPsifd,
				mPsikd;

		CalculateAuxiliarVariables(0);

		K_DP << K, Matrix::Zero(3, 3),
				Matrix::Zero(3, 3), K;

		mDVabc = K_DP*mIabc + E_r_vbr_DP;
		mVabc = dq0ToAbcTransform(mThetaMech, mVd, mVq, mV0);
		mIabc = dq0ToAbcTransform(mThetaMech, mId, mIq, mI0);

		CalculateLandR(0, dt*mBase_OmElec);
}

void Components::DP::SynchronGeneratorVBRNew::step(SystemModel& system, Real time)
{
		auto start = std::chrono::high_resolution_clock::now();

		stepInPerUnit(system.getOmega(), system.getTimeStep(), time, system.getNumMethod());

		// Update current source accordingly
		if (mNode1 >= 0) {
				system.addCompToRightSideVector(mNode1, Complex(mISourceEq(0), mISourceEq(3)));
		}
		if (mNode2 >= 0) {
				system.addCompToRightSideVector(mNode2, Complex(mISourceEq(1), mISourceEq(4)));
		}
		if (mNode3 >= 0) {
				system.addCompToRightSideVector(mNode3, Complex(mISourceEq(2), mISourceEq(5)));
		}


		//Update Equivalent Resistance
		system.addRealToSystemMatrix(mNode1, mNode1, mConductanceMat(0, 0));
		system.addRealToSystemMatrix(mNode1, mNode2, mConductanceMat(0, 1));
		system.addRealToSystemMatrix(mNode1, mNode3, mConductanceMat(0, 2));
		system.addRealToSystemMatrix(mNode2, mNode1, mConductanceMat(1, 0));
		system.addRealToSystemMatrix(mNode2, mNode2, mConductanceMat(1, 1));
		system.addRealToSystemMatrix(mNode2, mNode3, mConductanceMat(1, 2));
		system.addRealToSystemMatrix(mNode3, mNode1, mConductanceMat(2, 0));
		system.addRealToSystemMatrix(mNode3, mNode2, mConductanceMat(2, 1));
		system.addRealToSystemMatrix(mNode3, mNode3, mConductanceMat(2, 2));

		system.addRealToSystemMatrix(mNode1 + system.getCompOffset(), mNode1, mConductanceMat(3, 0));
		system.addRealToSystemMatrix(mNode1 + system.getCompOffset(), mNode2, mConductanceMat(3, 1));
		system.addRealToSystemMatrix(mNode1 + system.getCompOffset(), mNode3, mConductanceMat(3, 2));
		system.addRealToSystemMatrix(mNode2 + system.getCompOffset(), mNode1, mConductanceMat(4, 0));
		system.addRealToSystemMatrix(mNode2 + system.getCompOffset(), mNode2, mConductanceMat(4, 1));
		system.addRealToSystemMatrix(mNode2 + system.getCompOffset(), mNode3, mConductanceMat(4, 2));
		system.addRealToSystemMatrix(mNode3 + system.getCompOffset(), mNode1, mConductanceMat(5, 0));
		system.addRealToSystemMatrix(mNode3 + system.getCompOffset(), mNode2, mConductanceMat(5, 1));
		system.addRealToSystemMatrix(mNode3 + system.getCompOffset(), mNode3, mConductanceMat(5, 2));

		system.addRealToSystemMatrix(mNode1, mNode1 + system.getCompOffset(), mConductanceMat(0, 3));
		system.addRealToSystemMatrix(mNode1, mNode2 + system.getCompOffset(), mConductanceMat(0, 4));
		system.addRealToSystemMatrix(mNode1, mNode3 + system.getCompOffset(), mConductanceMat(0, 5));
		system.addRealToSystemMatrix(mNode2, mNode1 + system.getCompOffset(), mConductanceMat(1, 3));
		system.addRealToSystemMatrix(mNode2, mNode2 + system.getCompOffset(), mConductanceMat(1, 4));
		system.addRealToSystemMatrix(mNode2, mNode3 + system.getCompOffset(), mConductanceMat(1, 5));
		system.addRealToSystemMatrix(mNode3, mNode1 + system.getCompOffset(), mConductanceMat(2, 3));
		system.addRealToSystemMatrix(mNode3, mNode2 + system.getCompOffset(), mConductanceMat(2, 4));
		system.addRealToSystemMatrix(mNode3, mNode3 + system.getCompOffset(), mConductanceMat(2, 5));

		system.addRealToSystemMatrix(mNode1 + system.getCompOffset(), mNode1 + system.getCompOffset(), mConductanceMat(3, 3));
		system.addRealToSystemMatrix(mNode1 + system.getCompOffset(), mNode2 + system.getCompOffset(), mConductanceMat(3, 4));
		system.addRealToSystemMatrix(mNode1 + system.getCompOffset(), mNode3 + system.getCompOffset(), mConductanceMat(3, 5));
		system.addRealToSystemMatrix(mNode2 + system.getCompOffset(), mNode1 + system.getCompOffset(), mConductanceMat(4, 3));
		system.addRealToSystemMatrix(mNode2 + system.getCompOffset(), mNode2 + system.getCompOffset(), mConductanceMat(4, 4));
		system.addRealToSystemMatrix(mNode2 + system.getCompOffset(), mNode3 + system.getCompOffset(), mConductanceMat(4, 5));
		system.addRealToSystemMatrix(mNode3 + system.getCompOffset(), mNode1 + system.getCompOffset(), mConductanceMat(5, 3));
		system.addRealToSystemMatrix(mNode3 + system.getCompOffset(), mNode2 + system.getCompOffset(), mConductanceMat(5, 4));
		system.addRealToSystemMatrix(mNode3 + system.getCompOffset(), mNode3 + system.getCompOffset(), mConductanceMat(5, 5));
		
		system.updateLuFactored();

		auto finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = finish - start;
		Real StepDuration = elapsed.count();

		
		
		if (!system.MeasuringTime)
		{
				if (mLogLevel != Logger::Level::NONE) {
						Matrix logValues(getStatorCurrents().rows() + 3, 1);
						logValues << getStatorCurrents()*mBase_i, getElectricalTorque(), getRotationalSpeed(), StepDuration;
						mLog->LogGenDP(time, logValues);
				}
		}
		else
		{
				Matrix logValues(1, 1);
				logValues << StepDuration;
				mLog->LogGenShortDP(time, logValues);
		}


}

void Components::DP::SynchronGeneratorVBRNew::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
{

		// Calculate mechanical variables with euler
		if (mHasTurbineGovernor == true) {
				mMechTorque = -mTurbineGovernor.step(mOmMech, 1, 0.001, dt);
		}

		mElecTorque = (mPsimd*mIq - mPsimq*mId);
		mOmMech = mOmMech + dt * (1. / (2. * mH) * (mElecTorque - mMechTorque));
		mThetaMech = mThetaMech + dt * ((mOmMech - 1) * mBase_OmMech);
		mThetaMech2 = mThetaMech2 + dt * (mOmMech* mBase_OmMech);

		mPsikq1kq2 <<
				mPsikq1,
				mPsikq2;
		mPsifdkd <<
				mPsifd,
				mPsikd;

		CalculateLandR(time, dt*mBase_OmElec);

		CalculateAuxiliarVariables(time);

		
		E_r_vbr_DP = dq0ToAbcTransform(mThetaMech, h_qdr(1), h_qdr(0), 0);
		K_DP << K, Matrix::Zero(3, 3),
				Matrix::Zero(3, 3), K;

		R_eq_DP = Var1 + K_DP;
		E_eq_DP = Var2*mIabc + mDVabc - mVabc + E_r_vbr_DP;

		mConductanceMat = (R_eq_DP*mBase_Z).inverse();
		mISourceEq = R_eq_DP.inverse()*E_eq_DP*mBase_i;


}


void Components::DP::SynchronGeneratorVBRNew::postStep(SystemModel& system)
{
		Real dt = system.getTimeStep();

		if (mNode1 >= 0) {
				mVaRe = system.getCompFromLeftSideVector(mNode1).real();
				mVaIm = system.getCompFromLeftSideVector(mNode1).imag();
		}
		else {
				mVaRe = 0;
				mVaIm = 0;
		}
		if (mNode2 >= 0) {
				mVbRe = system.getCompFromLeftSideVector(mNode2).real();
				mVbIm = system.getCompFromLeftSideVector(mNode2).imag();
		}
		else {
				mVbRe = 0;
				mVbIm = 0;
		}
		if (mNode3 >= 0) {
				mVcRe = system.getCompFromLeftSideVector(mNode3).real();
				mVcIm = system.getCompFromLeftSideVector(mNode3).imag();
		}
		else {
				mVcRe = 0;
				mVcIm = 0;
		}


		mVabc <<
				mVaRe / mBase_v,
				mVbRe / mBase_v,
				mVcRe / mBase_v,
				mVaIm / mBase_v,
				mVbIm / mBase_v,
				mVcIm / mBase_v;

		mVq = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(0);
		mVd = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(1);
		mV0 = abcToDq0Transform(mThetaMech, mVaRe, mVbRe, mVcRe, mVaIm, mVbIm, mVcIm)(2);

		if (mHasExciter == true) {
				mVfd = mExciter.step(mVd, mVq, 1, dt);
		}

		mIabc = R_eq_DP.inverse()*(mVabc - E_eq_DP);

		mIaRe = mIabc(0);
		mIbRe = mIabc(1);
		mIcRe = mIabc(2);
		mIaIm = mIabc(3);
		mIbIm = mIabc(4);
		mIcIm = mIabc(5);
		mIq_hist = mIq;
		mId_hist = mId;
		mIq = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(0);
		mId = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(1);
		mI0 = abcToDq0Transform(mThetaMech, mIaRe, mIbRe, mIcRe, mIaIm, mIbIm, mIcIm)(2);

		// Calculate rotor flux likanges
		if (mNumDampingWindings == 2)
		{

				mDqStatorCurrents <<
						mIq,
						mId;

				mPsikq1kq2 = E1*mIq + E2*mPsikq1kq2 + E1*mIq_hist;
				mPsifdkd = F1*mId + F2*mPsifdkd + F1*mId_hist + F3*mVfd;

				mPsikq1 = mPsikq1kq2(0);
				mPsikq2 = mPsikq1kq2(1);
				mPsifd = mPsifdkd(0);
				mPsikd = mPsifdkd(1);

		}
		else
		{

				mDqStatorCurrents <<
						mIq,
						mId;

				mPsikq1 = E1_1d*mIq + E2_1d*mPsikq1 + E1_1d*mIq_hist;
				mPsifdkd = F1*mId + F2*mPsifdkd + F1*mId_hist + F3*mVfd;

				mPsifd = mPsifdkd(0);
				mPsikd = mPsifdkd(1);
		}

		// Calculate dynamic flux likages
		if (mNumDampingWindings == 2) {
				mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
		}
		else {
				mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
		}

		mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

		mDVabc = K_DP*mIabc + E_r_vbr_DP;
}


void Components::DP::SynchronGeneratorVBRNew::CalculateAuxiliarVariables(Real time) {

		if (mNumDampingWindings == 2)
		{
				c21_omega = -mOmMech*mDLmq / mLlkq1;
				c22_omega = -mOmMech*mDLmq / mLlkq2;
				c13_omega = mOmMech*mDLmd / mLlfd;
				c14_omega = mOmMech*mDLmd / mLlkd;

				K1a <<
						c11, c12,
						c21_omega, c22_omega;
				K1b <<
						c15,
						0;
				K1 = K1a*E1 + K1b;
		}

		else
		{
				c21_omega = -mOmMech*mDLmq / mLlkq1;
				c13_omega = mOmMech*mDLmd / mLlfd;
				c14_omega = mOmMech*mDLmd / mLlkd;

				K1a <<
						c11,
						c21_omega;
				K1b <<
						c15,
						0;
				K1 = K1a*E1_1d + K1b;
		}

		K2a <<
				c13_omega, c14_omega,
				c23, c24;
		K2b <<
				0,
				c25;
		K2 = K2a*F1 + K2b;

		K <<
				K1, K2, Matrix::Zero(2, 1),
				0, 0, 0;

		mKrs_teta <<
				2. / 3. * cos(mThetaMech2), 2. / 3. * cos(mThetaMech2 - 2. * M_PI / 3.), 2. / 3. * cos(mThetaMech2 + 2. * M_PI / 3.),
				2. / 3. * sin(mThetaMech2), 2. / 3. * sin(mThetaMech2 - 2. * M_PI / 3.), 2. / 3. * sin(mThetaMech2 + 2. * M_PI / 3.),
				1. / 3., 1. / 3., 1. / 3.;

		mKrs_teta_inv <<
				cos(mThetaMech2), sin(mThetaMech2), 1.,
				cos(mThetaMech2 - 2. * M_PI / 3.), sin(mThetaMech2 - 2. * M_PI / 3.), 1.,
				cos(mThetaMech2 + 2. * M_PI / 3.), sin(mThetaMech2 + 2. * M_PI / 3.), 1.;

		K = mKrs_teta_inv*K*mKrs_teta;

		if (mNumDampingWindings == 2)
				h_qdr = K1a*E2*mPsikq1kq2 + K1a*E1*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;
		else
				h_qdr = K1a*E2_1d*mPsikq1 + K1a*E1_1d*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;


}


void Components::DP::SynchronGeneratorVBRNew::CalculateAuxiliarConstants(Real dt) {

		b11 = (mRkq1 / mLlkq1)*(mDLmq / mLlkq1 - 1);
		b13 = mRkq1*mDLmq / mLlkq1;
		b31 = (mRfd / mLlfd)*(mDLmd / mLlfd - 1);
		b32 = mRfd*mDLmd / (mLlfd*mLlkd);
		b33 = mRfd*mDLmd / mLlfd;
		b41 = mRkd*mDLmd / (mLlfd*mLlkd);
		b42 = (mRkd / mLlkd)*(mDLmd / mLlkd - 1);
		b43 = mRkd*mDLmd / mLlkd;

		c23 = mDLmd*mRfd / (mLlfd*mLlfd)*(mDLmd / mLlfd - 1) + mDLmd*mDLmd*mRkd / (mLlkd*mLlkd*mLlfd);
		c24 = mDLmd*mRkd / (mLlkd*mLlkd)*(mDLmd / mLlkd - 1) + mDLmd*mDLmd*mRfd / (mLlfd*mLlfd*mLlkd);
		c25 = (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd;
		c26 = mDLmd / mLlfd;

		if (mNumDampingWindings == 2)
		{

				b12 = mRkq1*mDLmq / (mLlkq1*mLlkq2);
				b21 = mRkq2*mDLmq / (mLlkq1*mLlkq2);
				b22 = (mRkq2 / mLlkq2)*(mDLmq / mLlkq2 - 1);
				b23 = mRkq2*mDLmq / mLlkq2;
				c11 = mDLmq*mRkq1 / (mLlkq1*mLlkq1)*(mDLmq / mLlkq1 - 1) + mDLmq*mDLmq*mRkq2 / (mLlkq2*mLlkq2*mLlkq1);
				c12 = mDLmq*mRkq2 / (mLlkq2*mLlkq2)*(mDLmq / mLlkq2 - 1) + mDLmq*mDLmq*mRkq1 / (mLlkq1*mLlkq1*mLlkq2);
				c15 = (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq;


				Ea <<
						2 - dt*b11, -dt*b12,
						-dt*b21, 2 - dt*b22;
				E1b <<
						dt*b13,
						dt*b23;
				E1 = Ea.inverse() * E1b;

				E2b <<
						2 + dt*b11, dt*b12,
						dt*b21, 2 + dt*b22;
				E2 = Ea.inverse() * E2b;
		}

		else
		{

				c11 = mDLmq*mRkq1 / (mLlkq1*mLlkq1)*(mDLmq / mLlkq1 - 1);
				c15 = (mRkq1 / (mLlkq1*mLlkq1))*mDLmq*mDLmq;

				E1_1d = (1 / (2 - dt*b11))*dt*b13;
				E2_1d = (1 / (2 - dt*b11))*(2 + dt*b11);

		}


		Fa <<
				2 - dt*b31, -dt*b32,
				-dt*b41, 2 - dt*b42;
		F1b <<
				dt*b33,
				dt*b43;
		F1 = Fa.inverse() * F1b;

		F2b <<
				2 + dt*b31, dt*b32,
				dt*b41, 2 + dt*b42;

		F2 = Fa.inverse() * F2b;

		F3b <<
				2 * dt,
				0;
		F3 = Fa.inverse()* F3b;

		C26 <<
				0,
				c26;

}

void Components::DP::SynchronGeneratorVBRNew::CalculateLandR(Real time, Real dt)
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
		
		Real b_Re = cos(2 * mOmMech* mBase_OmMech*time);
		Real b_Im = sin(2 * mOmMech* mBase_OmMech*time);
		Real c_Re = cos(2 * mThetaMech2 - 2 * 1 * mBase_OmMech*time - 2 * mTheta0);
		Real c_Im = sin(2 * mThetaMech2 - 2 * 1 * mBase_OmMech*time - 2 * mTheta0);

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

		A = -L_EQ.inverse()*R_EQ;
		B = L_EQ.inverse();
		Var1 = B.inverse()*((2 / dt) * Matrix::Identity(6, 6) - A);
		Var2 =  -B.inverse()*((2 / dt) * Matrix::Identity(6, 6) + A);

}


Matrix Components::DP::SynchronGeneratorVBRNew::abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm)
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

Matrix Components::DP::SynchronGeneratorVBRNew::dq0ToAbcTransform(Real theta, Real d, Real q, Real zero)
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


