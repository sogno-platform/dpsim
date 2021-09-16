/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>
#include <cps/Solver/PFSolverInterfaceBus.h>
#include <cps/SP/SP_Ph1_PVNode.h>
#include <cps/SP/SP_Ph1_PQNode.h>
#include <cps/SP/SP_Ph1_VDNode.h>
#include <cps/Solver/MNAInterface.h>
#include <cps/SP/SP_Ph1_Capacitor.h>
#include <cps/SP/SP_Ph1_Inductor.h>
#include <cps/SP/SP_Ph1_Resistor.h>
#include <cps/PowerProfile.h>

namespace CPS {
namespace SP { namespace Ph1 {
	class Load :
		public SimPowerComp<Complex>,
		public SharedFactory<Load>,
		public PFSolverInterfaceBus,
		public MNAInterface{
	private:
		/// Power [Watt]
		Complex mPower;
		/// Nominal voltage [V]
		Real mNomVoltage;

		/// Active power [Watt]
		Real mActivePower;
		/// Reactive power [VAr]
		Real mReactivePower;

		/// base apparent power[VA]
		Real mBaseApparentPower;
		///base omega [1/s]
		Real mBaseOmega;
		/// Active power [pu]
		Real mActivePowerPerUnit;
		/// Reactive power [pu]
		Real mReactivePowerPerUnit;

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
		std::shared_ptr<SP::Ph1::Inductor> mSubInductor;
		/// Internal capacitor
		std::shared_ptr<SP::Ph1::Capacitor> mSubCapacitor;
		/// Internal resistance
		std::shared_ptr<SP::Ph1::Resistor> mSubResistor;
	public:
		/// Defines UID, name and logging level
		Load(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		/// Defines name and logging level
		Load(String name, Logger::Level logLevel = Logger::Level::off)
			: Load(name, name, logLevel) { }
		///
		void setParameters(Real activePower, Real reactivePower, Real nominalVoltage);
		///
		SimPowerComp<Complex>::Ptr clone(String name) override;

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromNodesAndTerminals(Real frequency) override;
		/// Load profile data
		PowerProfile mLoadProfile;
		/// Use the assigned load profile
		bool use_profile = false;
		/// Update PQ for this load for power flow calculation at next time step
		void updatePQ(Real time);

        // #### Powerflow section ####
		/// Calculates component's parameters in specified per-unit system
		void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
        /// Modify powerflow bus type
		void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) override;
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix) override;
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector) override;
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector) override;

		class MnaPostStep : public Task {
		public:
			MnaPostStep(Load& load, Attribute<Matrix>::Ptr leftVector) :
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
			Load& mLoad;
			Attribute<Matrix>::Ptr mLeftVector;
		};
	};
}
}

}
