/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/DP/DP_Ph1_CurrentSource.h>
#include <dpsim-models/PowerProfile.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// TODO: read from CSV files
	/// \brief PQ-load represented by a current source
	class PQLoadCS :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<PQLoadCS> {
	protected:
		/// Internal current source
		std::shared_ptr<DP::Ph1::CurrentSource> mSubCurrentSource;
		void updateSetPoint();
		void updateIntfValues();
	public:
		const Attribute<Real>::Ptr mActivePower;
		const Attribute<Real>::Ptr mReactivePower;
		const Attribute<Real>::Ptr mNomVoltage;

		/// Defines UID, name and logging level
		PQLoadCS(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines UID, name and logging level
		PQLoadCS(String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		PQLoadCS(String name,
			Real activePower, Real reactivePower, Real volt,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines UID, name and logging level
		PQLoadCS(String uid, String name,
			Real activePower, Real reactivePower, Real nomVolt,
			Logger::Level logLevel = Logger::Level::off);

		void setParameters(Real activePower, Real reactivePower, Real nomVolt);
		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);

		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;


		class MnaPreStep : public Task {
		public:
			MnaPreStep(PQLoadCS& load) :
				Task(**load.mName + ".MnaPreStep"), mLoad(load) {
				mLoad.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}

			void execute(Real time, Int timeStepCount);

		private:
			PQLoadCS& mLoad;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(PQLoadCS& load, Attribute<Matrix>::Ptr leftVector) :
				Task(**load.mName + ".MnaPostStep"), mLoad(load), mLeftVector(leftVector) {
				mLoad.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, leftVector);
			}

			void execute(Real time, Int timeStepCount);

		private:
			PQLoadCS& mLoad;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
