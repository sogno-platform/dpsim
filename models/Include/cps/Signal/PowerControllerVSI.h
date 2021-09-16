/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <cps/SimPowerComp.h>
#include <cps/SimSignalComp.h>
#include <cps/Task.h>

namespace CPS {
namespace Signal {
	class PowerControllerVSI :
		public SimSignalComp,
		public SharedFactory<PowerControllerVSI> {

	protected:
		/// Simulation time step
		Real mTimeStep;

		// Power parameters
		Real mPref = 0;
		Real mQref = 0;

		// Power controller
		Real mOmegaCutoff;
		Real mKiPowerCtrld;
		Real mKiPowerCtrlq;
		Real mKpPowerCtrld;
		Real mKpPowerCtrlq;

		// Current controller
		Real mKiCurrCtrld;
		Real mKiCurrCtrlq;
		Real mKpCurrCtrld;
		Real mKpCurrCtrlq;

		/// initial values for states
		Real mPInit = 0;
		Real mQInit = 0;
		Real mPhi_dInit = 0;
		Real mPhi_qInit = 0;
		Real mGamma_dInit = 0;
		Real mGamma_qInit = 0;

		// input, state and output vectors
		/// Previous Input
        Matrix mInputPrev = Matrix::Zero(6,1);
        /// Current Input
        Matrix mInputCurr = Matrix::Zero(6,1);
        /// Previous State
        Matrix mStatePrev = Matrix::Zero(6,1);
        /// Current State
        Matrix mStateCurr = Matrix::Zero(6,1);
        /// Previous Output
        Matrix mOutputPrev = Matrix::Zero(2,1);
        /// Current Output
        Matrix mOutputCurr = Matrix::Zero(2,1);

		// state space matrices
		/// matrix A of state space model
		Matrix mA = Matrix::Zero(6, 6);
		/// matrix B of state space model
		Matrix mB = Matrix::Zero(6, 6);
		/// matrix C of state space model
		Matrix mC = Matrix::Zero(2, 6);
		/// matrix D of state space model
		Matrix mD = Matrix::Zero(2, 6);

	public:
		PowerControllerVSI(String name, Logger::Level logLevel = Logger::Level::off);

		/// Setter for general parameters
		void setParameters(Real Pref, Real Qref);
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff);
		/// Setter for initial state values
		void setInitialStateValues(Real pInit, Real qInit, Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);

		/// Initialize vectors of state space model
		void initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Update B matrix due to its dependence on the input
		void updateBMatrixStateSpaceModel();

		/// pre step operations
		void signalPreStep(Real time, Int timeStepCount);
		/// pre step dependencies
		void signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// step operations
		void signalStep(Real time, Int timeStepCount);
		/// add step dependencies
		void signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		Task::List getTasks();

		class PreStep : public Task {
        public:
			PreStep(PowerControllerVSI& powerControllerVSI) :
                Task(powerControllerVSI.mName + ".PreStep"), mPowerControllerVSI(powerControllerVSI) {
					mPowerControllerVSI.signalAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mPowerControllerVSI.signalPreStep(time, timeStepCount); };
		private:
			PowerControllerVSI& mPowerControllerVSI;
        };
		class Step : public Task {
		public:
			Step(PowerControllerVSI& powerControllerVSI) :
				Task(powerControllerVSI.mName + ".Step"), mPowerControllerVSI(powerControllerVSI) {
					mPowerControllerVSI.signalAddStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mPowerControllerVSI.signalStep(time, timeStepCount); };
		private:
			PowerControllerVSI& mPowerControllerVSI;
		};
	};
}
}
