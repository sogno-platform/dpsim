/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/DP/DP_Ph1_varResSwitch.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief
	/// TBD
	
	class TriggeredSwitch :
		public varResSwitch,
		public SharedFactory<TriggeredSwitch> {
	
	protected:
        Real mSwitchClosedStartTime = 0.0;
        Real mSwitchClosedDuration = 5.0;

        Bool mExceedsThreholdPrev = false;
        Bool mExceedsThrehold = false;

        Real mTriggerThreshold = 0.5;

	public:
		
		/// Defines UID, name and log level
		TriggeredSwitch(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and log level
		TriggeredSwitch(String name, Logger::Level logLevel = Logger::Level::off)
			: TriggeredSwitch(name, name, logLevel) { }

		// #### MNA section ####
		/// Initializes MNA specific variables
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);

	class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(TriggeredSwitch& TriggeredSwitch) :
				Task(TriggeredSwitch.mName + ".MnaPreStep"), mTriggeredSwitch(TriggeredSwitch) {
					mTriggeredSwitch.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mTriggeredSwitch.mnaPreStep(time, timeStepCount); };

		private:
			TriggeredSwitch& mTriggeredSwitch;
	};

	};
}
}
}