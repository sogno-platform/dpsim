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


namespace CPS {

namespace SP {
namespace Ph1 {

        class SynchronGenerator : 
			public PowerComponent<Complex>,
			public SharedFactory<SynchronGenerator>,
			public PFSolverInterfaceBus {
	    private:			
			/// Rate apparent power [VA]
		    Real mRatedApparentPower;
			/// Rated voltage [V]
		    Real mRatedVoltage;

			/// Active power set point of the machine [W]
			Real mSetPointActivePower;
			/// Voltage set point of the machine [V]
			Real mSetPointVoltage;
			/// Maximum reactive power [VAr]
			Real mMaximumReactivePower;

			/// Base apparent power[VA]
			Real mBaseApparentPower;
			/// Base omega [1/s]
			Real mBaseOmega;
			/// Base voltage [V]
			Real mBaseVoltage;
			/// Active power set point of the machine [pu]
			Real mSetPointActivePowerPerUnit;
			/// Voltage set point of the machine [pu]
			Real mSetPointVoltagePerUnit;


        public:
			/// Defines UID, name and logging level
			SynchronGenerator(String uid, String name, Logger::Level logLevel = Logger::Level::off);
			/// Defines name and logging level
			SynchronGenerator(String name, Logger::Level logLevel = Logger::Level::off)
				: SynchronGenerator(name, name, logLevel) { }
			/// Setter for synchronous generator parameters
			void setParameters(Real ratedApparentPower, Real ratedVoltage, Real setPointActivePower, Real setPointVoltage, Real maximumReactivePower, PowerflowBusType powerflowBusType);

			// #### Powerflow section ####
			/// Set base voltage
			void setBaseVoltage(Real baseVoltage);
			/// Initializes component from power flow data
			void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
            /// Modify powerflow bus type
			void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
		};
}
}
}
