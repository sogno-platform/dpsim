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
#include <cps/DP/DP_Ph1_Capacitor.h>
#include <cps/DP/DP_Ph1_Inductor.h>
#include <cps/DP/DP_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// Constant impedance load model consisting of RLC elements
	class RXLoad :
		public SimPowerComp<Complex>,
		public MNAInterface,
		public SharedFactory<RXLoad> {
	protected:
		/// Power [Watt]
		Complex mPower;
		/// Active power [Watt]
		Real mActivePower;
		/// Reactive power [VAr]
		Real mReactivePower;
		/// Nominal voltage [V]
		Real mNomVoltage;
		/// Actual voltage [V]
		Complex mVoltage;
		/// Actual voltage [V]
		Complex mCurrent;
		/// Resistance [Ohm]
		Real mResistance;
		/// Reactance [Ohm]
		Real mReactance;
		/// Inductance [H]
		Real mInductance;
		/// Capacitance [F]
		Real mCapacitance;
		/// Internal inductor
		std::shared_ptr<DP::Ph1::Inductor> mSubInductor;
		/// Internal capacitor
		std::shared_ptr<DP::Ph1::Capacitor> mSubCapacitor;
		/// Internal resistance
		std::shared_ptr<DP::Ph1::Resistor> mSubResistor;
		/// Right side vectors of subcomponents
		std::vector<const Matrix*> mRightVectorStamps;
	public:
		/// Defines UID, name and logging level
		RXLoad(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RXLoad(String name,
			Logger::Level logLevel = Logger::Level::off);

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initialize component from power flow data
		void initializeFromNodesAndTerminals(Real frequency);
		/// Set model specific parameters
		void setParameters(Real activePower, Real ReactivePower, Real volt);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		/// Update interface current from MNA system result
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Update interface voltage from MNA system result
		void mnaUpdateVoltage(const Matrix& leftVector);
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

		class MnaPreStep : public Task {
		public:
			MnaPreStep(RXLoad& load) :
				Task(load.mName + ".MnaPreStep"), mLoad(load) {
				mLoad.mnaAddPreStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
			}
			void execute(Real time, Int timeStepCount) { mLoad.mnaPreStep(time, timeStepCount); }
		private:
			RXLoad& mLoad;
		};

		class MnaPostStep : public Task {
		public:
			MnaPostStep(RXLoad& load, Attribute<Matrix>::Ptr leftVector) :
				Task(load.mName + ".MnaPostStep"), mLoad(load), mLeftVector(leftVector) {
					mLoad.mnaAddPostStepDependencies(mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes, mLeftVector);
			}
			void execute(Real time, Int timeStepCount) { mLoad.mnaPostStep(time, timeStepCount, mLeftVector); }
		private:
			RXLoad& mLoad;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
