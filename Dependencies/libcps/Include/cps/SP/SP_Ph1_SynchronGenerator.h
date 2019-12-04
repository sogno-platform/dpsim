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


namespace CPS {

namespace SP {
namespace Ph1 {

        class SynchronGenerator : public PowerComponent<Complex>,
        public SharedFactory<SynchronGenerator>, public PFSolverInterfaceBus {
	    private:
		    Real mRatedU;
		    Real mRatedS;

        public:
			using PowerComponent<Complex>::PowerComponent;

            /// Defines UID, name, component parameters and logging level
			SynchronGenerator(String uid, String name,
				PowerflowBusType powerflowBusType,
				Logger::Level logLevel = Logger::Level::off);

			SynchronGenerator(String uid, String name, Real power,
				Real voltageSetPoint,PowerflowBusType powerflowBusType,
				Logger::Level logLevel = Logger::Level::off);

			SynchronGenerator(String uid, String name, Real power, Real maxQ, Real voltageSetPoint,
				Real ratedU, Real ratedS, PowerflowBusType powerflowBusType,
				Logger::Level logLevel = Logger::Level::off);

            // #### Powerflow section ####
            /// Modify powerflow bus type
			void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
			};
}
}
}
