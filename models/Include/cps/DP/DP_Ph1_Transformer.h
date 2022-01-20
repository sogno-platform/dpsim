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
#include <cps/DP/DP_Ph1_Resistor.h>
#include <cps/DP/DP_Ph1_Capacitor.h>
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
		/// Internal resistor to model losses
		std::shared_ptr<DP::Ph1::Resistor> mSubResistor;
		/// Internal inductor to model losses
		std::shared_ptr<DP::Ph1::Inductor> mSubInductor;

		/// Internal parallel resistance 1 as snubber
		std::shared_ptr<DP::Ph1::Resistor> mSubSnubResistor1;
		/// Internal parallel resistance 2 as snubber
		std::shared_ptr<DP::Ph1::Resistor> mSubSnubResistor2;
		/// Internal parallel capacitance 1 as snubber
		std::shared_ptr<DP::Ph1::Capacitor> mSubSnubCapacitor1;
		/// Internal parallel capacitance 2 as snubber
		std::shared_ptr<DP::Ph1::Capacitor> mSubSnubCapacitor2;

		/// Snubber resistance 1 [Ohm]
		Real mSnubberResistance1;
		/// Snubber resistance 2 [Ohm]
		Real mSnubberResistance2;
		/// Snubber capacitance 1 [F]
		Real mSnubberCapacitance1;
		/// Snubber capacitance 2 [F]
		Real mSnubberCapacitance2;

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
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);
		/// Set transformer specific parameters
		void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs, Real ratioPhase, Real resistance, Real inductance);
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);

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
		/// MNA pre step operations
		void mnaPreStep(Real time, Int timeStepCount);
		/// MNA post step operations
		void mnaPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector);
		/// Add MNA pre step dependencies
		void mnaAddPreStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes);
		/// Add MNA post step dependencies
		void mnaAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(Transformer& transformer) :
				Task(**transformer.mName + ".MnaPreStep"), mTransformer(transformer) {
					mTransformer.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mTransformer.mnaPreStep(time, timeStepCount); };
		private:
			Transformer& mTransformer;
		};


		class MnaPostStep : public Task {
		public:
			MnaPostStep(Transformer& transformer, Attribute<Matrix>::Ptr leftVector) :
				Task(**transformer.mName + ".MnaPostStep"), mTransformer(transformer), mLeftVector(leftVector) {
					mTransformer.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mTransformer.mnaPostStep(time, timeStepCount, mLeftVector); };

		private:
			Transformer& mTransformer;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
