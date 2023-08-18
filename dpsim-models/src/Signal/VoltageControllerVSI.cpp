/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/Signal/VoltageControllerVSI.h>

using namespace CPS;
using namespace CPS::Signal;

//Constructor
VoltageControllerVSI::VoltageControllerVSI(String name, Logger::Level logLevel) :
	SimSignalComp(name, name, logLevel),

	//Previous
	mInputPrev(mAttributes->create<Matrix>("input_prev", Matrix::Zero(6,1))),
    mStatePrev(mAttributes->create<Matrix>("state_prev", Matrix::Zero(4,1))),
    mOutputPrev(mAttributes->create<Matrix>("output_prev", Matrix::Zero(2,1))),

	//Currently
    mInputCurr(mAttributes->create<Matrix>("input_curr", Matrix::Zero(6,1))),
    mStateCurr(mAttributes->create<Matrix>("state_curr", Matrix::Zero(4,1))),
    mOutputCurr(mAttributes->create<Matrix>("output_curr", Matrix::Zero(2,1))),

	//Measured values from filter
	mVc_d(mAttributes->createDynamic<Real>("Vc_d")),
	mVc_q(mAttributes->createDynamic<Real>("Vc_q")),
	mIrc_d(mAttributes->createDynamic<Real>("Irc_d")),
	mIrc_q(mAttributes->createDynamic<Real>("Irc_q")) {

	SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
}

//set desired parameters to achieve
void VoltageControllerVSI::setParameters(Real VdRef, Real VqRef) {
	mVdRef = VdRef;
	mVqRef = VqRef;

	SPDLOG_LOGGER_INFO(mSLog, "General Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "Voltage D Amplitude={} [V] Voltage Q Amplitude={} [V]", mVdRef, mVqRef);

}

//setter for controller parameters and setting up of system matrices
void VoltageControllerVSI::setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Kp_pll, Real Ki_pll, Real Omega_cutoff) {

	// Voltage Loop (First)
	mKiVoltageCtrld = Ki_voltageCtrl;
	mKiVoltageCtrlq = Ki_voltageCtrl;
	mKpVoltageCtrld = Kp_voltageCtrl;
	mKpVoltageCtrlq = Kp_voltageCtrl;

	// Current Loop (Second)
	mKiCurrCtrld = Ki_currCtrl;
	mKiCurrCtrlq = Ki_currCtrl;
	mKpCurrCtrld = Kp_currCtrl;
	mKpCurrCtrlq = Kp_currCtrl;

	// Frequency
	mOmegaCutoff = Omega_cutoff; 

	// log loop parameters
	SPDLOG_LOGGER_INFO(mSLog, "Control Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "Voltage Loop: K_i = {}, K_p = {}", Kp_voltageCtrl, Ki_voltageCtrl);
	SPDLOG_LOGGER_INFO(mSLog, "Current Loop: K_i = {}, K_p = {}", Kp_currCtrl, Ki_currCtrl);
	SPDLOG_LOGGER_INFO(mSLog, "PLL: K_p = {}, K_i = {}", Kp_pll, Ki_pll);
	SPDLOG_LOGGER_INFO(mSLog, "Cut-Off Frequency = {}", Omega_cutoff);  

    // Set state space matrices using controller parameters

	// [x] = [phid, phiq, gammad, gammaq]
	// [u] = [vdref, vqref, vdc, vqc, idc, idq]
	// [y] = [vdout, vqout]

	mA <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		mKiVoltageCtrld, 0, 0, 0,
		0, mKiVoltageCtrlq, 0, 0;

	mB <<
		1, 0, -1, 0, 0, 0,
		0, 1, 0, -1, 0, 0,
		mKpVoltageCtrld, 0, -mKpVoltageCtrld, 0, -1, 0,
		0, mKpVoltageCtrlq, 0, -mKpVoltageCtrlq, 0, -1;

	mC <<
		mKpCurrCtrld*mKiVoltageCtrld, 0, mKiCurrCtrld, 0,
		0, mKpCurrCtrlq*mKiVoltageCtrlq, 0, mKiCurrCtrlq;

	mD <<
		mKpCurrCtrld*mKpVoltageCtrld , 0, -mKpCurrCtrld*mKpVoltageCtrld + 1, 0, -mKpCurrCtrld, 0,
		0, mKpCurrCtrlq*mKpVoltageCtrlq, 0, -mKpCurrCtrlq*mKpVoltageCtrlq + 1, 0, -mKpCurrCtrlq; 

	// Log state-space matrices
	SPDLOG_LOGGER_INFO(mSLog, "State space matrices:");
    SPDLOG_LOGGER_INFO(mSLog, "A = \n{}", mA);
    SPDLOG_LOGGER_INFO(mSLog, "B = \n{}", mB);
    SPDLOG_LOGGER_INFO(mSLog, "C = \n{}", mC);
    SPDLOG_LOGGER_INFO(mSLog, "D = \n{}", mD);
}

//setter for controller parameters and setting up of system matrices with VCO
void VoltageControllerVSI::setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_nominal) {

	//Voltage Loop (First)
	mKiVoltageCtrld = Ki_voltageCtrl;
	mKiVoltageCtrlq = Ki_voltageCtrl;
	mKpVoltageCtrld = Kp_voltageCtrl;
	mKpVoltageCtrlq = Kp_voltageCtrl;

	//Current Loop (Second)
	mKiCurrCtrld = Ki_currCtrl;
	mKiCurrCtrlq = Ki_currCtrl;
	mKpCurrCtrld = Kp_currCtrl;
	mKpCurrCtrlq = Kp_currCtrl;

	//Frequency
	mOmegaCutoff = Omega_nominal; 

	//log loop parameters
	SPDLOG_LOGGER_INFO(mSLog, "Control Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "Voltage Loop: K_i = {}, K_p = {}", Kp_voltageCtrl, Ki_voltageCtrl);
	SPDLOG_LOGGER_INFO(mSLog, "Current Loop: K_i = {}, K_p = {}", Kp_currCtrl, Ki_currCtrl);
	SPDLOG_LOGGER_INFO(mSLog, "Cut-Off Frequency = {}", Omega_nominal);  

    // Set state space matrices using controller parameters

	// [x] = [phid, phiq, gammad, gammaq]
	// [u] = [vdref, vqref, vdc, vqc, idc, idq]
	// [y] = [vdout, vqout]

	mA <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		mKiVoltageCtrld, 0, 0, 0,
		0, mKiVoltageCtrlq, 0, 0;

	mB <<
		1, 0, -1, 0, 0, 0,
		0, 1, 0, -1, 0, 0,
		mKpVoltageCtrld, 0, -mKpVoltageCtrld, 0, -1, 0,
		0, mKpVoltageCtrlq, 0, -mKpVoltageCtrlq, 0, -1;

	mC <<
		mKpCurrCtrld*mKiVoltageCtrld, 0, mKiCurrCtrld, 0,
		0, mKpCurrCtrlq*mKiVoltageCtrlq, 0, mKiCurrCtrlq;

	mD <<
		mKpCurrCtrld*mKpVoltageCtrld , 0, -mKpCurrCtrld*mKpVoltageCtrld + 1, 0, -mKpCurrCtrld, 0,
		0, mKpCurrCtrlq*mKpVoltageCtrlq, 0, -mKpCurrCtrlq*mKpVoltageCtrlq + 1, 0, -mKpCurrCtrlq; 

	// Log state-space matrices
	SPDLOG_LOGGER_INFO(mSLog, "State space matrices:");
    SPDLOG_LOGGER_INFO(mSLog, "A = \n{}", mA);
    SPDLOG_LOGGER_INFO(mSLog, "B = \n{}", mB);
    SPDLOG_LOGGER_INFO(mSLog, "C = \n{}", mC);
    SPDLOG_LOGGER_INFO(mSLog, "D = \n{}", mD);
}

//setter for initial state values --> variables inside the system
void VoltageControllerVSI::setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit) {

	mPhi_dInit = phi_dInit;
	mPhi_qInit = phi_qInit;
	mGamma_dInit = gamma_dInit;
	mGamma_qInit = gamma_qInit;

	SPDLOG_LOGGER_INFO(mSLog, "Initial State Value Parameters:");
	SPDLOG_LOGGER_INFO(mSLog, "Phi_dInit = {}, Phi_qInit = {}", phi_dInit, phi_qInit);
	SPDLOG_LOGGER_INFO(mSLog, "Gamma_dInit = {}, Gamma_qInit = {}", gamma_dInit, gamma_qInit);
}

//Creating state space model out of the variables
void VoltageControllerVSI::initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {

	mTimeStep = timeStep;
	mOmegaCutoff = omega;

	// initialization of input --> [u]
	**mInputCurr << mVdRef, mVqRef, **mVc_d, **mVc_q, **mIrc_d, **mIrc_q;
	SPDLOG_LOGGER_INFO(mSLog, "Initialization of input: \n" + Logger::matrixToString(**mInputCurr));

	// initialization of states --> [x]
	**mStateCurr << mPhi_dInit, mPhi_qInit, mGamma_dInit, mGamma_qInit;
	SPDLOG_LOGGER_INFO(mSLog, "Initialization of states: \n" + Logger::matrixToString(**mStateCurr));

	// initialization of output --> [y]
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	SPDLOG_LOGGER_INFO(mSLog, "Initialization of output: \n" + Logger::matrixToString(**mOutputCurr));
}

void VoltageControllerVSI::signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
    prevStepDependencies.push_back(mInputCurr);
	prevStepDependencies.push_back(mOutputCurr);
	modifiedAttributes.push_back(mInputPrev);
    modifiedAttributes.push_back(mOutputPrev);
};

void VoltageControllerVSI::signalPreStep(Real time, Int timeStepCount) {
    **mInputPrev = **mInputCurr;
    **mStatePrev = **mStateCurr;
    **mOutputPrev = **mOutputCurr;
}

void VoltageControllerVSI::signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {
	attributeDependencies.push_back(mVc_d);
	attributeDependencies.push_back(mVc_q);
	attributeDependencies.push_back(mIrc_d);
	attributeDependencies.push_back(mIrc_q);
	modifiedAttributes.push_back(mInputCurr);
    modifiedAttributes.push_back(mOutputCurr);
};

//Calculation happens here
void VoltageControllerVSI::signalStep(Real time, Int timeStepCount) {

	// get current inputs
	**mInputCurr << mVdRef, mVqRef, **mVc_d, **mVc_q, **mIrc_d, **mIrc_q;
    SPDLOG_LOGGER_DEBUG(mSLog, "Time {}\n: inputCurr = \n{}\n , inputPrev = \n{}\n , statePrev = \n{}", time, **mInputCurr, **mInputPrev, **mStatePrev);

	// calculate new states
	**mStateCurr = Math::StateSpaceTrapezoidal(**mStatePrev, mA, mB, mTimeStep, **mInputCurr, **mInputPrev);
	SPDLOG_LOGGER_DEBUG(mSLog, "stateCurr = \n {}", **mStateCurr);

	// calculate new outputs
	**mOutputCurr = mC * **mStateCurr + mD * **mInputCurr;
	SPDLOG_LOGGER_DEBUG(mSLog, "Output values: outputCurr = \n{}", **mOutputCurr);
}

Task::List VoltageControllerVSI::getTasks() {
	return Task::List({std::make_shared<PreStep>(*this), std::make_shared<Step>(*this)});
}
