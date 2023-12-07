/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_VSIVoltageSourceInverterDQ.h>

using namespace CPS;

void Base::VSIVoltageSourceInverterDQ::setParameters(
	Real sysOmega, Real VdRef, Real VqRef) {

	mOmegaNom = sysOmega;
	mVdRef = VdRef;
	mVqRef = VqRef;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nGeneral Parameters:"
		"\n\tNominal Omega = {} [1/s]"
		"\n\tVdRef = {} [V] "
		"\n\tVqRef = {} [V]",
		mOmegaNom, mVdRef, mVqRef);
}

void Base::VSIVoltageSourceInverterDQ::setTransformerParameters(
	Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance) {

	mTransformerNominalVoltageEnd1 = nomVoltageEnd1;
	mTransformerNominalVoltageEnd2 = nomVoltageEnd2;
	mTransformerResistance = resistance;
	mTransformerInductance = inductance;
	mTransformerRatioAbs = ratioAbs;
	mTransformerRatioPhase = ratioPhase;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nTransformer Parameters:"
		"\n\tNominal Voltage End 1={} [V] Nominal Voltage End 2={} [V]"
		"\n\tRated Apparent Power = {} [VA]"
		"\n\tResistance={} [Ohm] Inductance={} [H]"
    	"\n\tTap Ratio={} [ ] Phase Shift={} [deg]", 
		mTransformerRatioAbs, mTransformerNominalVoltageEnd1, mTransformerNominalVoltageEnd2,
		mTransformerResistance, mTransformerInductance, mTransformerRatioPhase);
}

void Base::VSIVoltageSourceInverterDQ::setFilterParameters(
	Real Lf, Real Cf, Real Rf, Real Rc) {

	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nFilter Parameters:"
		"\n\tInductance Lf = {} [H]"
		"\n\tCapacitance Cf = {} [F]"
		"\n\tResistance Rf = {} [H]" 
		"\n\tResistance Rc = {} [F]",
		mLf, mCf, mRf, mRc);
}

void Base::VSIVoltageSourceInverterDQ::setControllerParameters(
	Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega) {
	
	mKiVoltageCtrl = Ki_voltageCtrl;
	mKiCurrCtrl = Ki_currCtrl;
	mKpVoltageCtrl = Kp_voltageCtrl;
	mKpCurrCtrl = Kp_currCtrl;
	mOmegaVSI = Omega;

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nControl Parameters:"
		"\nVoltage Loop: K_p = {}, K_i = {}"
		"\nCurrent Loop: K_p = {}, K_i = {}"
		"\nVCO: Omega_Nom = {}", 
		mKpVoltageCtrl, mKiVoltageCtrl,
		mKpCurrCtrl, mKiCurrCtrl,
		Omega);
}

void Base::VSIVoltageSourceInverterDQ::setInitialStateValues(
	Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	SPDLOG_LOGGER_INFO(mLogger, 
		"\nInitial State Value Parameters:"
		"\n\tPhi_dInit = {}"
		"\n\tPhi_qInit = {}"
		"\n\tGamma_dInit = {}"
		"\n\tGamma_qInit = {}",
		phi_dInit, phi_qInit, gamma_dInit, gamma_qInit);
}

void Base::VSIVoltageSourceInverterDQ::initializeControllerStates() {
	Complex Vsref_dq = Math::rotatingFrame2to1((**mVsref)(0,0), **mThetaInv, **mThetaSys);
	**mPhi_d = **mIfilter_d;
	//**mPhi_d = **mIfilter_d / mKiv;
	//**mPhi_d = (**mIfilter_d + mOmegaNom*mCf * **mVcap_q) / mKiv;
	**mPhi_q = **mIfilter_q;
	//**mPhi_q = **mIfilter_q / mKiv; 
	//**mPhi_q = (**mIfilter_q - mOmegaNom*mCf * **mVcap_d) / mKiv; 
	**mGamma_d = Vsref_dq.real() - **mVcap_d;
	//**mGamma_d = (Vsref_dq.real() - **mVcap_d) / mKic;
	//**mGamma_d = (Vsref_dq.real() + mOmegaNom*mLf * **mIfilter_q) / mKic;
	**mGamma_q = Vsref_dq.imag() - **mVcap_q;
	//**mGamma_q = (Vsref_dq.imag() - **mVcap_q) / mKic;
	//**mGamma_q = (Vsref_dq.imag() - mOmegaNom*mLf * **mIfilter_d) / mKic;

	SPDLOG_LOGGER_INFO(mSLog_,
			"\nInitialize controller states:"	  
			"\n\tPhi_d = {}"
			"\n\tPhi_q = {}"
			"\n\tGamma_d = {}"
			"\n\tGamma_q = {}",
			**mPhi_d, **mPhi_q, **mGamma_d, **mGamma_q);
	mSLog_->flush();
}