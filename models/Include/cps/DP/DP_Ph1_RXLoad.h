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
#include <cps/DP/DP_Ph1_Capacitor.h>
#include <cps/DP/DP_Ph1_Inductor.h>
#include <cps/DP/DP_Ph1_Resistor.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// \brief
	/// TODO: currently modelled as an impedance, which obviously doesn't have a constant power characteristic
	/// Model as current source and read from CSV files
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
		/// Conductance [S]
		Real mConductance;
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
	public:
		/// Defines UID, name and logging level
		RXLoad(String uid, String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RXLoad(String name,
			Logger::Level logLevel = Logger::Level::off);
		/// Defines name, component parameters and logging level
		RXLoad(String name,
			Real activePower, Real reactivePower, Real volt,
			Logger::Level logLevel = Logger::Level::off);

		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General ####
		///
		void initialize(Matrix frequencies);
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		/// Sets model specific parameters
		void setParameters(Real activePower, Real ReactivePower, Real volt);

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Stamps right side (source) vector
		void mnaApplyRightSideVectorStamp(Matrix& rightVector);

		void mnaUpdateCurrent(const Matrix& leftVector);
		void mnaUpdateVoltage(const Matrix& leftVector);

		class MnaPreStep : public Task {
		public:
			MnaPreStep(RXLoad& load) :
				Task(load.mName + ".MnaPreStep"), mLoad(load) {
				if (load.mSubResistor)
					mAttributeDependencies.push_back(load.mSubResistor->attribute("right_vector"));
				if (load.mSubInductor)
					mAttributeDependencies.push_back(load.mSubInductor->attribute("right_vector"));
				if (load.mSubCapacitor)
					mAttributeDependencies.push_back(load.mSubCapacitor->attribute("right_vector"));
				mModifiedAttributes.push_back(load.attribute("right_vector"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			RXLoad& mLoad;
		};


		class MnaPostStep : public Task {
		public:
			MnaPostStep(RXLoad& load, Attribute<Matrix>::Ptr leftVector) :
				Task(load.mName + ".MnaPostStep"), mLoad(load), mLeftVector(leftVector) {
				mAttributeDependencies.push_back(leftVector);
				if (load.mSubResistor)
					mAttributeDependencies.push_back(load.mSubResistor->attribute("i_intf"));
				if (load.mSubInductor)
					mAttributeDependencies.push_back(load.mSubInductor->attribute("i_intf"));
				if (load.mSubCapacitor)
					mAttributeDependencies.push_back(load.mSubCapacitor->attribute("i_intf"));
				mModifiedAttributes.push_back(load.attribute("i_intf"));
				mModifiedAttributes.push_back(load.attribute("v_intf"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			RXLoad& mLoad;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}
}
