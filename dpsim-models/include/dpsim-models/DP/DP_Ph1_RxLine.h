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
#include <dpsim-models/Base/Base_Ph1_PiLine.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph1 {

	class RxLine :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public Base::Ph1::PiLine,
		public SharedFactory<RxLine> {
	protected:
		/// Voltage across the component [V]
		/// FIXME: This is only read once, but never written to, so its always zero
		Complex mVoltage;
		/// Current through the component [A]
		/// FIXME: This is never used...
		Complex mCurrent;
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

		SimPowerComp<Complex>::Ptr clone(String name);

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
