/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Base/Base_AvVoltageSourceInverterDQ.h>

using namespace CPS;

Base::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(String uid, String name, Logger::Level logLevel)
: TopologicalPowerComp(uid, name, logLevel) {
    // state variables
	addAttribute<Real>("theta", &mThetaPLL, Flags::read | Flags::write);
	addAttribute<Real>("phipll", &mPhiPLL, Flags::read | Flags::write);
	addAttribute<Real>("p", &mP, Flags::read | Flags::write);
	addAttribute<Real>("q", &mQ, Flags::read | Flags::write);
	addAttribute<Real>("phid", &mPhi_d, Flags::read | Flags::write);
	addAttribute<Real>("phiq", &mPhi_q, Flags::read | Flags::write);
	addAttribute<Real>("gammad", &mGamma_d, Flags::read | Flags::write);
	addAttribute<Real>("gammaq", &mGamma_q, Flags::read | Flags::write);

	// input variables
	addAttribute<Real>("Omega_nom", &mOmegaN, Flags::read | Flags::write);
	addAttribute<Real>("P_ref", &mPref, Flags::read | Flags::write);
	addAttribute<Real>("Q_ref", &mQref, Flags::read | Flags::write);
}

void Base::AvVoltageSourceInverterDQ::setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
	Real ratioPhase, Real resistance, Real inductance, Real omega) {

	mTransformerResistance = resistance;
	mTransformerInductance = inductance;
	mTransformerRatioAbs = ratioAbs;
	mTransformerRatioPhase = ratioPhase;

	mSLog->info("Connection Transformer Parameters:");
	mSLog->info("Resistance={} [Ohm] Inductance={} [H]", mTransformerResistance, mTransformerInductance);
    mSLog->info("Tap Ratio={} [ ] Phase Shift={} [deg]", mTransformerRatioAbs, mTransformerRatioPhase);
};

void Base::AvVoltageSourceInverterDQ::setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref) {
	mPref = Pref;
	mQref = Qref;

	mVnom = sysVoltNom;
	mOmegaN = sysOmega;

	mSLog->info("General Parameters:");
	mSLog->info("Nominal Voltage={} [V] Nominal Omega={} [1/s]", mVnom, mOmegaN);
	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mPref, mQref);    

	// use Pref and Qref as init values for states P and Q
	// init values for other states remain zero (if not changed using setInitialStateValues)
	mPInit = Pref;
	mQInit = Qref;

	parametersSet = true;
}

void Base::AvVoltageSourceInverterDQ::setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {
	mKpPLL = Kp_pll;
	mKiPLL = Ki_pll;

	mKiPowerCtrld = Ki_powerCtrl;
	mKiPowerCtrlq = Ki_powerCtrl;

	mKpPowerCtrld = Kp_powerCtrl;
	mKpPowerCtrlq = Kp_powerCtrl;

	mKiCurrCtrld = Ki_currCtrl;
	mKiCurrCtrlq = Ki_currCtrl;

	mKpCurrCtrld = Kp_currCtrl;
	mKpCurrCtrlq = Kp_currCtrl;

	mOmegaCutoff = Omega_cutoff;

	mSLog->info("Control Parameters:");
	mSLog->info("PLL: K_i = {}, K_p = {}", mKpPLL, mKiPLL);
	mSLog->info("Power Loop: K_i = {}, K_p = {}", mKpPowerCtrld, mKiPowerCtrld);
	mSLog->info("Current Loop: K_i = {}, K_p = {}", mKpCurrCtrld, mKiCurrCtrld);
	mSLog->info("Cut-Off Frequency = {}", mOmegaCutoff);

    // Set state space matrices using controller parameters
	mA <<
		0, mKiPLL, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, -mOmegaCutoff, 0, 0, 0, 0, 0,
		0, 0, 0, -mOmegaCutoff, 0, 0, 0, 0,
		0, 0, -1, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, -mKpPowerCtrld, 0, mKiPowerCtrld, 0, 0, 0,
		0, 0, 0, mKpPowerCtrlq, 0, mKiPowerCtrlq, 0, 0;

	mB <<
		1, 0, 0, 0, mKpPLL, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, 0, 0,
		0, mKpPowerCtrld, 0, 0, 0, -1, 0,
		0, 0, -mKpPowerCtrlq, 0, 0, 0, -1;

	mC <<
		0, 0, -mKpPowerCtrld * mKpCurrCtrld, 0, mKpCurrCtrld * mKiPowerCtrld, 0, mKiCurrCtrld, 0,
		0, 0, 0, mKpPowerCtrlq * mKpCurrCtrlq, 0, mKpCurrCtrlq*mKiPowerCtrlq, 0, mKiCurrCtrlq;

	mD <<
		0, mKpCurrCtrld*mKpPowerCtrld, 0, 0, 0, -mKpCurrCtrld, 0,
		0, 0, -mKpCurrCtrlq * mKpPowerCtrlq, 0, 0, 0, -mKpCurrCtrlq;
}

void Base::AvVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc) {
	mLf = Lf;
	mCf = Cf;
	mRf = Rf;
	mRc = Rc;

	mSLog->info("Filter Parameters:");
	mSLog->info("Inductance Lf={} [H] Capacitance Cf={} [F]", mLf, mCf);
	mSLog->info("Resistance Rf={} [H] Resistance Rc={} [F]", mRf, mRc);
}

void Base::AvVoltageSourceInverterDQ::setInitialStateValues(Real thetaPLLInit, Real phiPLLInit, Real pInit, Real qInit,
	Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	mThetaPLLInit = thetaPLLInit;
	mPhiPLLInit = phiPLLInit;
	mPInit = pInit;
	mQInit = qInit;
	mPhi_dInit = phi_dInit;
	mPhi_qInit = phi_qInit;
	mGamma_dInit = gamma_dInit;
	mGamma_qInit = gamma_qInit;

	mSLog->info("Initial State Value Parameters:");
	mSLog->info("ThetaPLLInit = {}, PhiPLLInit = {}", mThetaPLLInit, mPhiPLLInit);
	mSLog->info("PInit = {}, QInit = {}", mPInit, mQInit);
	mSLog->info("Phi_dInit = {}, Phi_qInit = {}", mPhi_dInit, mPhi_qInit);
	mSLog->info("Gamma_dInit = {}, Gamma_qInit = {}", mGamma_dInit, mGamma_qInit);
};