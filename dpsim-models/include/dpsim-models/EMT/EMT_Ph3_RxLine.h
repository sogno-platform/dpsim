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
#include <dpsim-models/Base/Base_Ph3_PiLine.h>
#include <dpsim-models/EMT/EMT_Ph3_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

	class RxLine :
		public SimPowerComp<Real>,
		public MNAInterface,
		public Base::Ph3::PiLine,
		public SharedFactory<RxLine> {
	protected:
		/// Inductance submodel
		std::shared_ptr<Inductor> mSubInductor;
		/// Resistor submodel
		std::shared_ptr<Resistor> mSubResistor;
		/// Inductor end to ground resistor to facilitate initialization
		std::shared_ptr<Resistor> mInitialResistor;

	public:
		/// Defines UID, name, logging level
		RxLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RxLine(String name, Logger::Level logLevel = Logger::Level::off)
			: RxLine(name, name, logLevel) { }

		SimPowerComp<Real>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps system matrix
		void mnaApplyInitialSystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaUpdateVoltage(const Matrix& leftVector);
		void mnaUpdateCurrent(const Matrix& leftVector);

		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes) override;
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;

		class MnaPreStep : public Task {
		public:
			MnaPreStep(RxLine& line) :
				Task(**line.mName + ".MnaPreStep"), mLine(line) {
				mLine.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}

			void execute(Real time, Int timeStepCount);

		private:
			RxLine& mLine;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(RxLine& line, Attribute<Matrix>::Ptr leftVector) :
				Task(**line.mName + ".MnaPostStep"), mLine(line), mLeftVector(leftVector) {
				mLine.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, leftVector);
			}

			void execute(Real time, Int timeStepCount);

		private:
			RxLine& mLine;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
