/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/DP/DP_Ph1_RXLoad.h>
#include <cps/DP/DP_Ph1_Switch.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Constant impedance load model consisting of RLC elements
	class RXLoadSwitch :
		public SimPowerComp<Complex>,
		public MNASwitchInterface,
		public SharedFactory<RXLoadSwitch> {
	protected:
		/// Internal RXLoad
		std::shared_ptr<DP::Ph1::RXLoad> mSubRXLoad;
		/// Internal protection switch
		std::shared_ptr<DP::Ph1::Switch> mSubSwitch;
		/// internal switch is only opened after this time offset
		Real mSwitchTimeOffset = 1.0;
		/// Right side vectors of subcomponents
		std::vector<const Matrix*> mRightVectorStamps;

	public:
		/// Defines UID, name and logging level
		RXLoadSwitch(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RXLoadSwitch(String name, Logger::Level logLevel = Logger::Level::off);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Sets model specific parameters
		void setParameters(Real activePower, Real reactivePower, Real nomVolt,
			Real openResistance, Real closedResistance, Bool closed = false);
		/// Sets only switch parameters so that load parameters could be calculated from powerflow
		void setSwitchParameters(Real openResistance, Real closedResistance, Bool closed = false);
		/// built-in logic for protection switch
		void updateSwitchState(Real time);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector) { }
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector) { }
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
			AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
			AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes,
			Attribute<Matrix>::Ptr &leftVector);

		// #### MNA section for switch ####
		/// Check if switch is closed
		Bool mnaIsClosed() { return mSubSwitch->isClosed(); }
		/// Stamps system matrix considering the defined switch position
		void mnaApplySwitchSystemMatrixStamp(Bool closed, Matrix& systemMatrix, Int freqIdx);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(RXLoadSwitch& load) : Task(load.mName + ".MnaPreStep"), mLoad(load) {
				mLoad.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mLoad.mnaPreStep(time, timeStepCount); }
		private:
			RXLoadSwitch& mLoad;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(RXLoadSwitch& load, Attribute<Matrix>::Ptr leftVector) :
				Task(load.mName + ".MnaPostStep"), mLoad(load), mLeftVector(leftVector) {
					mLoad.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mLoad.mnaPostStep(time, timeStepCount, mLeftVector); }
		private:
			RXLoadSwitch& mLoad;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
