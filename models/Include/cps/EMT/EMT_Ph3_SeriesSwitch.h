/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNASwitchInterface.h>
#include <cps/Base/Base_Ph1_Switch.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// @brief EMT three-phase switch
	///
	/// The switch can be opened and closed.
	/// Each state has a specific resistance value.
	/// For this model, the resistance model is the
	/// same for all phases and only in series.
	class SeriesSwitch :
		public Base::Ph1::Switch,
		public SimPowerComp<Real>,
		public SharedFactory<SeriesSwitch>,
		public MNASwitchInterface {
	protected:
	public:
		/// Defines UID, name and log level
		SeriesSwitch(String uid, String name, Logger::Level loglevel = Logger::Level::off);
		/// Defines name and log level
		SeriesSwitch(String name, Logger::Level logLevel = Logger::Level::off)
			: SeriesSwitch(name, name, logLevel) { }

		// #### General ####
		/// Return new instance with the same parameters
		SimPowerComp<Real>::Ptr clone(String name);
		/// Initializes states from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### General MNA section ####
		/// Initializes MNA specific variables
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Update interface voltage from MNA system results
		void mnaUpdateVoltage(const Matrix& leftVector);
		/// Update interface voltage from MNA system results
		void mnaUpdateCurrent(const Matrix& leftVector);

		// #### Switch specific MNA section ####
		/// Check if switch is closed
		Bool mnaIsClosed() { return mIsClosed; }
		/// Stamps system matrix considering the defined switch position
		void mnaApplySwitchSystemMatrixStamp(Bool closed, Matrix& systemMatrix, Int freqIdx);

		class MnaPostStep : public Task {
		public:
			MnaPostStep(SeriesSwitch& sSwitch, Attribute<Matrix>::Ptr leftSideVector) :
				Task(sSwitch.mName + ".MnaPostStep"),
				mSwitch(sSwitch), mLeftVector(leftSideVector) {

				mAttributeDependencies.push_back(mLeftVector);
				mModifiedAttributes.push_back(mSwitch.attribute("v_intf"));
				mModifiedAttributes.push_back(mSwitch.attribute("i_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			SeriesSwitch& mSwitch;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
