/**
 * @file
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <cps/PowerComponent.h>
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
		public PowerComponent<Complex>,
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
		PowerComponent<Complex>::Ptr clone(String name);

		// #### General ####
		/// Initializes component from power flow data
		void initializeFromPowerflow(Real frequency);
		/// Load profile data
		PowerProfile mLoadProfile;
		/// Use the assigned load profile
		bool use_profile = false;
		/// Update PQ for this load for power flow calculation at next time step
		void updatePQ(Real time);

        // #### Powerflow section ####
		/// Initializes component from power flow data
		void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);		
        /// Modify powerflow bus type
		void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;

		// #### MNA section ####
		/// Initializes internal variables of the component
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);
		/// Stamps system matrix
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
		/// Updates internal current variable of the component
		void mnaUpdateCurrent(const Matrix& leftVector);
		/// Updates internal voltage variable of the component
		void mnaUpdateVoltage(const Matrix& leftVector);

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
