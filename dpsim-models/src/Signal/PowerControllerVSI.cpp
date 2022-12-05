/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/PowerControllerVSI.h>

using namespace CPS;
using namespace CPS::Signal;

PowerControllerVSI::PowerControllerVSI(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),
	/// CHECK: Which of these really need to be attributes?
	mInputPrev(attributeList->create<Matrix>("input_prev", Matrix::Zero(6,1))),
    mStatePrev(attributeList->create<Matrix>("state_prev", Matrix::Zero(6,1))),
    mOutputPrev(attributeList->create<Matrix>("output_prev", Matrix::Zero(2,1))),
    mInputCurr(attributeList->create<Matrix>("input_curr", Matrix::Zero(6,1))),
    mStateCurr(attributeList->create<Matrix>("state_curr", Matrix::Zero(6,1))),
    mOutputCurr(attributeList->create<Matrix>("output_curr", Matrix::Zero(2,1))),
	mVc_d(mAttributes->createDynamic<Real>("Vc_d")),
	mVc_q(mAttributes->createDynamic<Real>("Vc_q")),
	mIrc_d(mAttributes->createDynamic<Real>("Irc_d")),
	mIrc_q(mAttributes->createDynamic<Real>("Irc_q")) {

	mSLog->info("Create {} {}", type(), name);
}

void PowerControllerVSI::setParameters(Real Pref, Real Qref) {
	mPref = Pref;
	mQref = Qref;

	mSLog->info("General Parameters:");
	mSLog->info("Active Power={} [W] Reactive Power={} [VAr]", mPref, mQref);

	// use Pref and Qref as init values for states P and Q
	// init values for other states remain zero (if not changed using setInitialStateValues)
	mPInit = Pref;
	mQInit = Qref;
}

void PowerControllerVSI::setControllerParameters(Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {

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
	mSLog->info("Power Loop: K_i = {}, K_p = {}", Kp_powerCtrl, Ki_powerCtrl);
	mSLog->info("Current Loop: K_i = {}, K_p = {}", Kp_currCtrl, Ki_currCtrl);
	mSLog->info("Cut-Off Frequency = {}", Omega_cutoff);

    // Set state space matrices using controller parameters
	mA <<
		-mOmegaCutoff, 0, 0, 0, 0, 0,
		0, -mOmegaCutoff, 0, 0, 0, 0,
		-1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		-mKpPowerCtrld, 0, mKiPowerCtrld, 0, 0, 0,
		0, mKpPowerCtrlq, 0, mKiPowerCtrlq, 0, 0;

	mB <<
		0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 0,
		0, -1, 0, 0, 0, 0,
		mKpPowerCtrld, 0, 0, 0, -1, 0,
		0, -mKpPowerCtrlq, 0, 0, 0, -1;

	mC <<
		-mKpPowerCtrld * mKpCurrCtrld, 0, mKpCurrCtrld * mKiPowerCtrld, 0, mKiCurrCtrld, 0,
		0, mKpPowerCtrlq * mKpCurrCtrlq, 0, mKpCurrCtrlq*mKiPowerCtrlq, 0, mKiCurrCtrlq;

	mD <<
		mKpCurrCtrld*mKpPowerCtrld, 0, 0, 0, -mKpCurrCtrld, 0,
		0, -mKpCurrCtrlq * mKpPowerCtrlq, 0, 0, 0, -mKpCurrCtrlq;

	mSLog->info("State space matrices:");
    mSLog->info("A = \n{}", mA);
    mSLog->info("B = \n{}", mB);
    mSLog->info("C = \n{}", mC);
    mSLog->info("D = \n{}", mD);
}

void PowerControllerVSI::setInitialStateValues(Real pInit, Real qInit,
	Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	mPInit = pInit;
	mQInit = qInit;
	mPhi_dInit = phi_dInit;
	mPhi_qInit = phi_qInit;
	mGamma_dInit = gamma_dInit;
	mGamma_qInit = gamma_qInit;

	mSLog->info("Initial State Value Parameters:");
	mSLog->info("PInit = {}, QInit = {}", pInit, qInit);
	mSLog->info("Phi_dInit = {}, Phi_qInit = {}", phi_dInit, phi_qInit);
	mSLog->info("Gamma_dInit = {}, Gamma_qInit = {}", gamma_dInit, gamma_qInit);
}

void PowerControllerVSI::initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	mTimeStep = timeStep;
	mOmegaCutoff = omega;

	// update B matrix due to its dependence on Irc
	updateBMatrixStateSpaceModel();

	// initialization of input
	**mInputCurr << mPref, mQref, **mVc_d, **mVc_q, **mIrc_d, **mIrc_q;
	mSLog->info("Initialization of input: \n" + Logger::matrixToString(**mInputCurr));

	// initialization of states
	**mStateCurr << mPInit, mQInit, mPhi_dInit, mPhi_qInit, mGamma_dInit, mGamma_qInit;
	mSLog->info("Initialization of states: \n" + Logger::matrixToString(**mStateCurr));

	// initialization of output
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	mSLog->info("Initialization of output: \n" + Logger::matrixToString(**mOutputCurr));
}

void PowerControllerVSI::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void PowerControllerVSI::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void PowerControllerVSI::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mVc_d);
	attributeDependencies.push_back(mVc_q);
	attributeDependencies.push_back(mIrc_d);
	attributeDependencies.push_back(mIrc_q);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

void PowerControllerVSI::signalStep(Real time, Int timeStepCount) {
	// update B matrix due to its dependence on Irc
	updateBMatrixStateSpaceModel();

	// get current inputs
	**mInputCurr << mPref, mQref, **mVc_d, **mVc_q, **mIrc_d, **mIrc_q;
    mSLog->debug("Time {}\n: inputCurr = \n{}\n , inputPrev = \n{}\n , statePrev = \n{}", time, **mInputCurr, **mInputPrev, **mStatePrev);

	// calculate new states
	**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
	mSLog->debug("stateCurr = \n {}", **mStateCurr);

	// calculate new outputs
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	mSLog->debug("Output values: outputCurr = \n{}", **mOutputCurr);
}

void PowerControllerVSI::updateBMatrixStateSpaceModel() {
	mB.coeffRef(0, 2) = mOmegaCutoff * **mIrc_d;
	mB.coeffRef(0, 3) = mOmegaCutoff * **mIrc_q;
	mB.coeffRef(1, 2) = -mOmegaCutoff * **mIrc_q;
	mB.coeffRef(1, 3) = mOmegaCutoff * **mIrc_d;
}

Task::List PowerControllerVSI::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
