/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/Signal/PowerControllerVSI.h>

using namespace CPS;
using namespace CPS::Signal;

PowerControllerVSI::PowerControllerVSI(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel) {

	addAttribute<Real>("theta", &mThetaPLL, Flags::read | Flags::write);
	addAttribute<Real>("phipll", &mPhiPLL, Flags::read | Flags::write);
	addAttribute<Real>("p", &mP, Flags::read | Flags::write);
	addAttribute<Real>("q", &mQ, Flags::read | Flags::write);
	addAttribute<Real>("phid", &mPhi_d, Flags::read | Flags::write);
	addAttribute<Real>("phiq", &mPhi_q, Flags::read | Flags::write);
	addAttribute<Real>("gammad", &mGamma_d, Flags::read | Flags::write);
	addAttribute<Real>("gammaq", &mGamma_q, Flags::read | Flags::write);

	addAttribute<Real>("Vc_d", Flags::read | Flags::write);
	addAttribute<Real>("Vc_q", Flags::read | Flags::write);
	addAttribute<Real>("Irc_d", Flags::read | Flags::write);
	addAttribute<Real>("Irc_q", Flags::read | Flags::write);
	addAttribute<Real>("Vs_d", Flags::read | Flags::write);
	addAttribute<Real>("Vs_q", Flags::read | Flags::write);
}

void PowerControllerVSI::setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref) {
	mPref = Pref;
	mQref = Qref;

	mVnom = sysVoltNom;
	mOmegaN = sysOmega;

	// use Pref and Qref as init values for states P and Q
	// init values for other states remain zero (if not changed using setInitialStateValues)
	mPInit = Pref;
	mQInit = Qref;
}

void PowerControllerVSI::setControllerParameters(Real Kp_pll, Real Ki_pll,
	Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {

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

void PowerControllerVSI::setInitialStateValues(Real thetaPLLInit, Real phiPLLInit, Real pInit, Real qInit,
	Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	mThetaPLLInit = thetaPLLInit;
	mPhiPLLInit = phiPLLInit;
	mPInit = pInit;
	mQInit = qInit;
	mPhi_dInit = phi_dInit;
	mPhi_qInit = phi_qInit;
	mGamma_dInit = gamma_dInit;
	mGamma_qInit = gamma_qInit;
}

void PowerControllerVSI::initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;
	mOmegaN = omega;
	mOmegaCutoff = omega;

	// update B matrix due to its dependence on Irc
	updateBMatrixStateSpaceModel();

	// initialization of input
	mU << mOmegaN, mPref, mQref, attribute<Real>("Vc_d")->get(), attribute<Real>("Vc_q")->get(), attribute<Real>("Irc_d")->get(), attribute<Real>("Irc_q")->get();
	mSLog->info("Initialization of input: \n" + Logger::matrixToString(mU));

	// initialization of states
	mThetaPLL = mThetaPLLInit;
	// mThetaPLL = std::arg(mVirtualNodes[4]->initialSingleVoltage());
	mPhiPLL = mPhiPLLInit;
	mP = mPInit;
	mQ = mQInit;
	mPhi_d = mPhi_dInit;
	mPhi_q = mPhi_qInit;
	mGamma_d = mGamma_dInit;
	mGamma_q = mGamma_qInit;
	mStates << mThetaPLL, mPhiPLL, mP, mQ, mPhi_d, mPhi_q, mGamma_d, mGamma_q;
	mSLog->info("Initialization of states: \n" + Logger::matrixToString(mStates));

	// initialization of output
	Matrix vsdq = Matrix::Zero(2, 1);
	vsdq = mC * mStates + mD * mU;
	attribute<Real>("Vs_d")->set(vsdq(0,0));
	attribute<Real>("Vs_q")->set(vsdq(1,0));
	mSLog->info("Initialization of output: \n" + Logger::matrixToString(vsdq));
}

void PowerControllerVSI::signalStep(Real time, Int timeStepCount) {
	Matrix newStates = Matrix::Zero(8, 1);
	Matrix newU = Matrix::Zero(7, 1);

	newU << mOmegaN, mPref, mQref, attribute<Real>("Vc_d")->get(), attribute<Real>("Vc_q")->get(), attribute<Real>("Irc_d")->get(), attribute<Real>("Irc_q")->get();

	newStates = Math::StateSpaceTrapezoidal(mStates, mA, mB, mTimeStep, newU, mU);

	// update states
	mThetaPLL = newStates(0, 0);
	mPhiPLL = newStates(1, 0);
	mP = newStates(2, 0);
	mQ = newStates(3, 0);
	mPhi_d = newStates(4, 0);
	mPhi_q = newStates(5, 0);
	mGamma_d = newStates(6, 0);
	mGamma_q = newStates(7, 0);

	mStates = newStates;
	mU = newU;

	// new output
	Matrix vsdq = Matrix::Zero(2, 1);
	vsdq = mC * mStates + mD * mU;
	attribute<Real>("Vs_d")->set(vsdq(0,0));
	attribute<Real>("Vs_q")->set(vsdq(1,0));
}

void PowerControllerVSI::updateBMatrixStateSpaceModel() {
	mB.coeffRef(2, 3) = mOmegaCutoff * attribute<Real>("Irc_d")->get();
	mB.coeffRef(2, 4) = mOmegaCutoff * attribute<Real>("Irc_q")->get();
	mB.coeffRef(3, 3) = -mOmegaCutoff * attribute<Real>("Irc_q")->get();
	mB.coeffRef(3, 4) = mOmegaCutoff * attribute<Real>("Irc_d")->get();
}

Task::List PowerControllerVSI::getTasks() {
	return Task::List({std::make_shared<Step>(*this)});
}