/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/DP/DP_Ph1_VoltageSource.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Ideal voltage source representing a synchronous generator
	class SynchronGeneratorIdeal :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<SynchronGeneratorIdeal> {
	private:
		/// Inner voltage source that represents the generator
		std::shared_ptr<DP::Ph1::VoltageSource> mSubVoltageSource;
	public:
		/// Voltage set point [V]
		/// CHECK: Is this attribute necessary? It is not used in the component itself, only initialized
		const Attribute<Complex>::Ptr mVoltageRef;
		/// Defines UID, name, component parameters and logging level
		SynchronGeneratorIdeal(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		SynchronGeneratorIdeal(String name,
			Logger::Level logLevel = Logger::Level::off);

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);
		/// Updates current through the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates voltage across component
		void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPreStep : public CPS::Task {
		public:
			MnaPreStep(SynchronGeneratorIdeal& SynchronGeneratorIdeal) :
				Task(**SynchronGeneratorIdeal.mName + ".MnaPreStep"), mSynchronGeneratorIdeal(SynchronGeneratorIdeal) {
					mSynchronGeneratorIdeal.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mSynchronGeneratorIdeal.mnaPreStep(time, timeStepCount); };

		private:
			SynchronGeneratorIdeal& mSynchronGeneratorIdeal;
		};

		class MnaPostStep : public CPS::Task {
		public:
			MnaPostStep(SynchronGeneratorIdeal& SynchronGeneratorIdeal, Attribute<Matrix>::Ptr leftVector) :
				Task(**SynchronGeneratorIdeal.mName + ".MnaPostStep"), mSynchronGeneratorIdeal(SynchronGeneratorIdeal), mLeftVector(leftVector) {
				mSynchronGeneratorIdeal.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mSynchronGeneratorIdeal.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			SynchronGeneratorIdeal& mSynchronGeneratorIdeal;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
