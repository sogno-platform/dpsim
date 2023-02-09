/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/Solver/PFSolverInterfaceBus.h>
#include <dpsim-models/SP/SP_Ph1_PVNode.h>
#include <dpsim-models/SP/SP_Ph1_PQNode.h>
#include <dpsim-models/SP/SP_Ph1_VDNode.h>
#include <dpsim-models/SP/SP_Ph1_Capacitor.h>
#include <dpsim-models/SP/SP_Ph1_Inductor.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>
#include <dpsim-models/PowerProfile.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	class Load :
		public CompositePowerComp<Complex>,
		public SharedFactory<Load>,
		public PFSolverInterfaceBus {
	public:
		/// Nominal voltage [V]
		const Attribute<Real>::Ptr mNomVoltage;
		/// Active power [Watt]
		const Attribute<Real>::Ptr mActivePower;
		/// Reactive power [VAr]
		const Attribute<Real>::Ptr mReactivePower;
		/// Active power [pu]
		const Attribute<Real>::Ptr mActivePowerPerUnit;
		/// Reactive power [pu]
		const Attribute<Real>::Ptr mReactivePowerPerUnit;

	private:
		/// base apparent power[VA]
		Real mBaseApparentPower;
		///base omega [1/s]
		Real mBaseOmega;
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
		void setParameters(Real activePower, Real reactivePower, Real nominalVoltage=0);
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
		/// Updates internal current variable of the component
		void mnaCompUpdateCurrent(const Matrix& leftVector) override;
		/// Updates internal voltage variable of the component
		void mnaCompUpdateVoltage(const Matrix& leftVector) override;

		/// MNA post step operations
		void mnaParentPostStep(Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) override;

		void mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies, AttributeBase::List &attributeDependencies, AttributeBase::List &modifiedAttributes, Attribute<Matrix>::Ptr &leftVector) override;
	};
}
}

}
