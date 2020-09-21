/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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
		Real mPref;
		Real mQref;

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

		// states
		Real mP;
		Real mQ;
		Real mPhi_d;
		Real mPhi_q;
		Real mGamma_d;
		Real mGamma_q;

		/// initial values for states
		Real mPInit = 0;
		Real mQInit = 0;
		Real mPhi_dInit = 0;
		Real mPhi_qInit = 0;
		Real mGamma_dInit = 0;
		Real mGamma_qInit = 0;

		/// state space matrices
		Matrix mA = Matrix::Zero(6, 6);
		Matrix mB = Matrix::Zero(6, 6);
		Matrix mC = Matrix::Zero(2, 6);
		Matrix mD = Matrix::Zero(2, 6);

		/// state vector
		Matrix mStates = Matrix::Zero(6, 1);

		/// input vector
		Matrix mU = Matrix::Zero(6, 1);
		
	public:
		PowerControllerVSI(String name, Logger::Level logLevel = Logger::Level::off);
		
		/// Setter for general parameters
		void setParameters(Real Pref, Real Qref);
		/// Setter for parameters of control loops
		void setControllerParameters(Real Kp_powerCtrl, Real Ki_powerCtrl, Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff);
		/// Setter for initial state values
		void setInitialStateValues(Real pInit, Real qInit, Real phi_dInit, Real phi_qInit, Real gamma_dInit, Real gamma_qInit);

		///
		void initializeStateSpaceModel(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		///
		void updateBMatrixStateSpaceModel();

		/// step operations
		void signalStep(Real time, Int timeStepCount);
		/// add step dependencies
		void signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) {};

		Task::List getTasks();

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
