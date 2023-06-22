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
	class VCO :
		public SimSignalComp,
		public SharedFactory<VCO> {

	protected:
		
		/// Nominal frequency
		Real mOmegaNom;
		/// Integration time step
        Real mTimeStep;

		/// matrix A of state space model
		Matrix mA = Matrix::Zero(2, 2);
		/// matrix B of state space model
		Matrix mB = Matrix::Zero(2, 2);
		/// matrix C of state space model
		Matrix mC = Matrix::Zero(2, 2);
		/// matrix D of state space model
		Matrix mD = Matrix::Zero(2, 2);

	public:

		/// This is never explicitely set to reference anything, so the outside code is responsible for setting up the reference.
		const Attribute<Real>::Ptr mInputRef;

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

		VCO(String name, Logger::Level logLevel = Logger::Level::off);
		/// Setter for VCO parameters
		void setParameters(Real omegaNom);
		/// Setter for simulation parameters
		void setSimulationParameters(Real timestep);
		/// Setter for initial values
        void setInitialValues(Real input_init, Matrix state_init, Matrix output_init);
		/// Composition of A, B, C, D matrices based on VCO parameters
		void composeStateSpaceMatrices();
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
			PreStep(VCO& VCO) :
                Task(**VCO.mName + ".PreStep"), mVCO(VCO) {
					mVCO.signalAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVCO.signalPreStep(time, timeStepCount); };
		private:
			VCO& mVCO;
        };

		class Step : public Task {
		public:
			Step(VCO& VCO) :
				Task(**VCO.mName + ".Step"), mVCO(VCO) {
					mVCO.signalAddStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mVCO.signalStep(time, timeStepCount); };
		private:
			VCO& mVCO;
		};
	};
}
}
