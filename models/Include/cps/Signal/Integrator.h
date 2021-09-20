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
	class Integrator :
		public SimSignalComp,
		public SharedFactory<Integrator> {

	protected:
		/// Integration time step
        Real mTimeStep;
        /// Previous Input
        Real mInputPrev;
        /// Current Input
        Real mInputCurr;
        /// Previous State
        Real mStatePrev;
        /// Current State
        Real mStateCurr;
        /// Previous Output
        Real mOutputPrev;
        /// Current Output
        Real mOutputCurr;

	public:
		Integrator(String name, Logger::Level logLevel = Logger::Level::off);
		/// Setter for integration step parameter
		void setParameters(Real timestep);
		/// Setter for initial values
        void setInitialValues(Real input_init, Real state_init, Real output_init);
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
			PreStep(Integrator& integrator) :
                Task(integrator.mName + ".PreStep"), mIntegrator(integrator) {
					mIntegrator.signalAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mIntegrator.signalPreStep(time, timeStepCount); };
		private:
			Integrator& mIntegrator;
        };

		class Step : public Task {
		public:
			Step(Integrator& integrator) :
				Task(integrator.mName + ".Step"), mIntegrator(integrator) {
					mIntegrator.signalAddStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mIntegrator.signalStep(time, timeStepCount); };
		private:
			Integrator& mIntegrator;
		};
	};
}
}
