/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
	class Droop :
		public SimSignalComp,
		public SharedFactory<Droop> {

	protected:

		// Inverter parameters
		Real mOmegaNom;
		Real mPowerSet;

		// Controller parameters
		Real mM_p;
		Real mTau_p;
		Real mTau_l;

		// Integration time step
        Real mTimeStep;

		/// matrix A of state space model
		Matrix mA = Matrix::Zero(1, 1);
		/// matrix B of state space model
		Matrix mB = Matrix::Zero(1, 3);
		/// matrix C of state space model
		Matrix mC = Matrix::Zero(1, 1);
		/// matrix D of state space model
		Matrix mD = Matrix::Zero(1, 3);

	public:

		/// This is never explicitely set to reference anything, so the outside code is responsible for setting up the reference.
		const Attribute<Real>::Ptr mInputRef;

		const Attribute<Real>::Ptr mOutputRef; // to the VCO input
		
		/// Previous Input
        const Attribute<Matrix>::Ptr mInputPrev;
        /// Current Input
        const Attribute<Matrix>::Ptr mInputCurr;
        /// Previous State
        const Attribute<Matrix>::Ptr mStatePrev;
        /// Current State
        const Attribute<Matrix>::Ptr mStateCurr;
        /// Previous Output
        const Attribute<Matrix>::Ptr mOutputPrev;
        /// Current Output
        const Attribute<Matrix>::Ptr mOutputCurr;

		Droop(String name, Logger::Level logLevel = Logger::Level::off);
		/// Setter for Droop parameters
		void setParameters(Real powerSet, Real omegaNom);
		/// Setter for simulation parameters
		void setControllerParameters(Real m_p, Real tau_p, Real tau_l);
		/// Setter for initial values
        void setSimulationParameters(Real timestep);
		/// Composition of A, B, C, D matrices based on Droop parameters
		void setInitialStateValues(Matrix input_init, Matrix state_init, Matrix output_init);
		/// pre step operations
		void signalPreStep(Real time, Int timeStepCount);
		/// step operations
		void signalStep(Real time, Int timeStepCount);
		/// pre step dependencies
		void signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// add step dependencies
		void signalAddStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

		Task::List getTasks();

        class PreStep : public Task {
        public:
			PreStep(Droop& Droop) :
                Task(**Droop.mName + ".PreStep"), mDroop(Droop) {
					mDroop.signalAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mDroop.signalPreStep(time, timeStepCount); };
		private:
			Droop& mDroop;
        };

		class Step : public Task {
		public:
			Step(Droop& Droop) :
				Task(**Droop.mName + ".Step"), mDroop(Droop) {
					mDroop.signalAddStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mDroop.signalStep(time, timeStepCount); };
		private:
			Droop& mDroop;
		};
	};
}
}
