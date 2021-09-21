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
#include <cps/Base/Base_Ph3_PiLine.h>
#include <cps/EMT/EMT_Ph3_Inductor.h>
#include <cps/EMT/EMT_Ph3_Resistor.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

	class RxLine :
		public SimPowerComp<Real>,
		public MNAInterface,
		public Base::Ph3::PiLine,
		public SharedFactory<RxLine> {
	protected:
		/// Voltage across the component [V]
		Matrix mVoltage;
		/// Current through the component [A]
		Matrix mCurrent;
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

		class MnaPreStep : public Task {
		public:
			MnaPreStep(RxLine& line) :
				Task(line.mName + ".MnaPreStep"), mLine(line) {
				mAttributeDependencies.push_back(line.mSubResistor->attribute("right_vector"));
				mAttributeDependencies.push_back(line.mSubInductor->attribute("right_vector"));
				mModifiedAttributes.push_back(line.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			RxLine& mLine;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(RxLine& line, Attribute<Matrix>::Ptr leftVector) :
				Task(line.mName + ".MnaPostStep"), mLine(line), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				mAttributeDependencies.push_back(line.mSubInductor->attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("i_intf"));
				mModifiedAttributes.push_back(line.attribute("v_intf"));
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
