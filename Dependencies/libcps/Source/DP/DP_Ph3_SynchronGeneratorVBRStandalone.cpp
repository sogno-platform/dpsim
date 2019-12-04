/** Voltage behind reactance (DP)
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

#include <cps/DP/DP_Ph3_SynchronGeneratorVBRStandalone.h>

using namespace CPS;

DP::Ph3::SynchronGeneratorVBRStandalone::SynchronGeneratorVBRStandalone(String name,
	Real nomPower, Real nomVolt, Real nomFreq, Int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia, Logger::Level logLevel)
	: SynchronGeneratorBase(name, nomPower, nomVolt, nomFreq, poleNumber, nomFieldCur,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2,
		inertia, logLevel)
{
}

void DP::Ph3::SynchronGeneratorVBRStandalone::addExciter(Real Ta, Real Ka, Real Te, Real Ke, Real Tf, Real Kf, Real Tr, Real Lad, Real Rfd)
{
	mExciter = Exciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lad, Rfd);
	mExciter.initialize(1, 1);

	mHasExciter = true;
}

void DP::Ph3::SynchronGeneratorVBRStandalone::addGovernor(Real Ta, Real Tb, Real Tc, Real Fa, Real Fb, Real Fc, Real K, Real Tsr, Real Tsm, Real Tm_init, Real PmRef)
{
	mTurbineGovernor = TurbineGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm);
	mTurbineGovernor.initialize(PmRef, Tm_init);
	mHasTurbineGovernor = true;
}

void DP::Ph3::SynchronGeneratorVBRStandalone::initialize(Real om, Real dt,
	Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle, Real initFieldVoltage, Real initMechPower) {

	mSystemOmega = om;
	mSystemTimeStep = dt;

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

	CalculateAuxiliarConstants(dt*mBase_OmElec);

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

void DP::Ph3::SynchronGeneratorVBRStandalone::mnaStep(Matrix& systemMatrix, Matrix& rightVector, Matrix& leftVector, Real time) {
	stepInPerUnit(mSystemOmega, mSystemTimeStep, time, mNumericalMethod);

	R_load = systemMatrix.inverse() / mBase_Z;

	// Update current source accordingly
	if ( terminalNotGrounded(0) ) {
		Math::addToVectorElement(rightVector, simNode(0), Complex(-mIaRe*mBase_I,  -mIaIm*mBase_i));
	}
	if ( terminalNotGrounded(1) ) {
		Math::addToVectorElement(rightVector, simNode(1), Complex(-mIbRe*mBase_I,  -mIbIm*mBase_i));
	}
	if ( simNode(2) >= 0) {
		Math::addToVectorElement(rightVector, simNode(2), Complex(-mIcRe*mBase_I,  -mIcIm*mBase_i));
	}

	if (mLogLevel != Logger::Level::off) {
		Matrix logValues(rotorFluxes().rows() + dqStatorCurrents().rows() + 3, 1);
		logValues << rotorFluxes(), dqStatorCurrents(), electricalTorque(), rotationalSpeed(), rotorPosition();
		mLog->debug(time, logValues);
	}
}

void DP::Ph3::SynchronGeneratorVBRStandalone::stepInPerUnit(Real om, Real dt, Real time, NumericalMethod numMethod)
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
		mIabc = Math::StateSpaceTrapezoidal(mIabc, -L_EQ.inverse()*(R_EQ + R_load), L_EQ.inverse(), dt*mBase_OmElec, -mDVabc, -mDVabc_hist);
	else
		mIabc = Math::StateSpaceEuler(mIabc, -L_EQ.inverse()*(R_EQ + R_load), L_EQ.inverse(), dt*mBase_OmElec, -mDVabc);

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

	mPsikq1kq2 <<
			mPsikq1,
			mPsikq2;
	mPsifdkd <<
			mPsifd,
			mPsikd;

		// Calculate rotor flux likanges
		if (mNumDampingWindings == 2) {
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
		else {
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
				mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
				mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
		}
		else {
				mDPsiq = mDLmq*(mPsikq1 / mLlkq1);
				mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mIq);
		}
		mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
		mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

		CalculateAuxiliarVariables(time);

		Matrix mDVqd = Matrix::Zero(2, 1);
		mDVqd = K*mDqStatorCurrents + h_qdr;

		mDVd = mDVqd(1);
		mDVq = mDVqd(0);

		mDVabc = K_DP*mIabc + E_r_vbr_DP;

		//mDVaRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(0);
		//mDVbRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(1);
		//mDVcRe = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(2);
		//mDVaIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(3);
		//mDVbIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(4);
		//mDVcIm = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0)(5);

		mDVabc_hist = mDVabc;

		mDVabc = dq0ToAbcTransform(mThetaMech, mDVd, mDVq, 0);
		//mDVabc <<
		//		mDVaRe,
		//		mDVbRe,
		//		mDVcRe,
		//		mDVaIm,
		//		mDVbIm,
		//		mDVcIm;
}

void DP::Ph3::SynchronGeneratorVBRStandalone::CalculateLandR(Real time) {
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

void DP::Ph3::SynchronGeneratorVBRStandalone::CalculateAuxiliarConstants(Real dt) {

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

void DP::Ph3::SynchronGeneratorVBRStandalone::CalculateAuxiliarVariables(Real time) {

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

		K << K1, K2;

		//K <<
		//		K1, K2, Matrix::Zero(2, 1),
		//		0, 0, 0;

		//mKrs_teta <<
		//		2. / 3. * cos(mThetaMech), 2. / 3. * cos(mThetaMech - 2. * M_PI / 3.), 2. / 3. * cos(mThetaMech + 2. * M_PI / 3.),
		//		2. / 3. * sin(mThetaMech), 2. / 3. * sin(mThetaMech - 2. * M_PI / 3.), 2. / 3. * sin(mThetaMech + 2. * M_PI / 3.),
		//		1. / 3., 1. / 3., 1. / 3.;

		//mKrs_teta_inv <<
		//		cos(mThetaMech), sin(mThetaMech), 1.,
		//		cos(mThetaMech - 2. * M_PI / 3.), sin(mThetaMech - 2. * M_PI / 3.), 1,
		//		cos(mThetaMech + 2. * M_PI / 3.), sin(mThetaMech + 2. * M_PI / 3.), 1.;

		//K = mKrs_teta_inv*K*mKrs_teta;

		if (mNumDampingWindings == 2)
				h_qdr = K1a*E2*mPsikq1kq2 + K1a*E1*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;
		else
				h_qdr = K1a*E2_1d*mPsikq1 + K1a*E1_1d*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;

		//H_qdr << h_qdr,
		//		0;

		//E_r_vbr = mKrs_teta_inv*H_qdr;


}

Matrix DP::Ph3::SynchronGeneratorVBRStandalone::abcToDq0Transform(Real theta, Real aRe, Real bRe, Real cRe, Real aIm, Real bIm, Real cIm) {
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

Matrix DP::Ph3::SynchronGeneratorVBRStandalone::dq0ToAbcTransform(Real theta, Real d, Real q, Real zero)
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
