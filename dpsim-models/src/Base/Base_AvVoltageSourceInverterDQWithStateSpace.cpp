/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Base/Base_AvVoltageSourceInverterDQWithStateSpace.h>

using namespace CPS;

void Base::AvVoltageSourceInverterDQWithStateSpace::setTransformerParameters(
    Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
    Real ratioPhase, Real resistance, Real inductance, Real omega) {

  mTransformerResistance = resistance;
  mTransformerInductance = inductance;
  mTransformerRatioAbs = ratioAbs;
  mTransformerRatioPhase = ratioPhase;
};

void Base::AvVoltageSourceInverterDQWithStateSpace::setParameters(
    Real sysOmega, Real sysVoltNom, Real Pref, Real Qref) {
  mPref = Pref;
  mQref = Qref;

  mVnom = sysVoltNom;
  mOmegaN = sysOmega;

  // use Pref and Qref as init values for states P and Q
  // init values for other states remain zero (if not changed using setInitialStateValues)
  mPInit = Pref;
  mQInit = Qref;
}

void Base::AvVoltageSourceInverterDQWithStateSpace::setControllerParameters(
    Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl,
    Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {

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

  // Set state space matrices using controller parameters
  mA << 0, mKiPLL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      -mOmegaCutoff, 0, 0, 0, 0, 0, 0, 0, 0, -mOmegaCutoff, 0, 0, 0, 0, 0, 0,
      -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -mKpPowerCtrld, 0,
      mKiPowerCtrld, 0, 0, 0, 0, 0, 0, mKpPowerCtrlq, 0, mKiPowerCtrlq, 0, 0;

  mB << 1, 0, 0, 0, mKpPLL, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0,
      mKpPowerCtrld, 0, 0, 0, -1, 0, 0, 0, -mKpPowerCtrlq, 0, 0, 0, -1;

  mC << 0, 0, -mKpPowerCtrld * mKpCurrCtrld, 0, mKpCurrCtrld * mKiPowerCtrld, 0,
      mKiCurrCtrld, 0, 0, 0, 0, mKpPowerCtrlq * mKpCurrCtrlq, 0,
      mKpCurrCtrlq * mKiPowerCtrlq, 0, mKiCurrCtrlq;

  mD << 0, mKpCurrCtrld * mKpPowerCtrld, 0, 0, 0, -mKpCurrCtrld, 0, 0, 0,
      -mKpCurrCtrlq * mKpPowerCtrlq, 0, 0, 0, -mKpCurrCtrlq;
}

void Base::AvVoltageSourceInverterDQWithStateSpace::setFilterParameters(
    Real Lf, Real Cf, Real Rf, Real Rc) {
  mLf = Lf;
  mCf = Cf;
  mRf = Rf;
  mRc = Rc;
}

void Base::AvVoltageSourceInverterDQWithStateSpace::setInitialStateValues(
    Real thetaPLLInit, Real phiPLLInit, Real pInit, Real qInit, Real phi_dInit,
    Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

  mThetaPLLInit = thetaPLLInit;
  mPhiPLLInit = phiPLLInit;
  mPInit = pInit;
  mQInit = qInit;
  mPhi_dInit = phi_dInit;
  mPhi_qInit = phi_qInit;
  mGamma_dInit = gamma_dInit;
  mGamma_qInit = gamma_qInit;
}
