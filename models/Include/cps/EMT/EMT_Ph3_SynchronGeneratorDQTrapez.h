/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/EMT/EMT_Ph3_SynchronGeneratorDQ.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	class SynchronGeneratorDQTrapez :
		public SynchronGeneratorDQ,
		public SharedFactory<SynchronGeneratorDQTrapez> {
	public:
		SynchronGeneratorDQTrapez(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		SynchronGeneratorDQTrapez(String name, Logger::Level loglevel = Logger::Level::off);

		// #### MNA Section ####
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(SynchronGeneratorDQTrapez& synGen)
				: Task(synGen.mName + ".MnaPreStep"), mSynGen(synGen) {
				mModifiedAttributes.push_back(synGen.attribute("right_vector"));
				mPrevStepDependencies.push_back(synGen.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SynchronGeneratorDQTrapez& mSynGen;
		};

	protected:
		// #### Trapezoidal Section ####

		/// Performs an Euler forward step with the state space model of a synchronous generator
		/// to calculate the flux and current from the voltage vector in per unit.
		void stepInPerUnit(Real time);
	};
}
}
}
