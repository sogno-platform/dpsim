/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
