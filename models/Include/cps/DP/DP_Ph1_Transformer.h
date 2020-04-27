/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/DP/DP_Ph1_RxLine.h>
#include <cps/DP/DP_Ph1_Inductor.h>
#include <cps/Base/Base_Ph1_Transformer.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Transformer that includes an inductance and resistance
	class Transformer :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<Transformer>,
		public Base::Ph1::Transformer {
	private:
		/// Internal inductor to model losses
		std::shared_ptr<DP::Ph1::Inductor> mSubInductor;
		/// Internal parallel resistance as snubber
		std::shared_ptr<DP::Ph1::Resistor> mSubSnubResistor;
		std::shared_ptr<DP::Ph1::Resistor> mSubResistor;

		/// Snubber resistance added on the low voltage side
		Real mSnubberResistance;

		/// Boolean for considering resistive losses with sub resistor
		Bool mWithResistiveLosses;
	public:
		/// Defines UID, name and logging level
		Transformer(String uid, String name,
			Logger::Level logLevel = Logger::Level::off, Bool withResistiveLosses = false);
		/// Defines name and logging level
		Transformer(String name, Logger::Level logLevel = Logger::Level::off)
			: Transformer(name, name, logLevel) { }

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Defines component parameters
		void setParameters(Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);
		///
		void initialize(Matrix frequencies);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Transformer& transformer) :
				Task(transformer.mName + ".MnaPreStep"), mTransformer(transformer) {
				mAttributeDependencies.push_back(transformer.mSubSnubResistor->attribute("right_vector"));
				mAttributeDependencies.push_back(transformer.mSubInductor->attribute("right_vector"));
				if (transformer.mSubResistor)
					mAttributeDependencies.push_back(transformer.mSubResistor->attribute("right_vector"));
				mModifiedAttributes.push_back(transformer.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Transformer& mTransformer;
		};


		class MnaPostStep : public Task {
		public:
			MnaPostStep(Transformer& transformer, Attribute<Matrix>::Ptr leftVector) :
				Task(transformer.mName + ".MnaPostStep"), mTransformer(transformer), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(transformer.mSubInductor->attribute("i_intf"));
				mAttributeDependencies.push_back(leftVector);
				mModifiedAttributes.push_back(transformer.attribute("i_intf"));
				mModifiedAttributes.push_back(transformer.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			Transformer& mTransformer;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
