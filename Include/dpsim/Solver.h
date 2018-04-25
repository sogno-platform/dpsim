/** Simulation
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
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

#include <iostream>
#include <vector>
#include <list>

#include "cps/Logger.h"
#include "cps/SystemTopology.h"
#include "cps/Interface.h"

using namespace CPS;

namespace DPsim {
	/// Holds switching time and which system should be activated.
	struct SwitchConfiguration {
		Real switchTime;
		UInt systemIndex;
	};

	class Solver {

	public:
		enum class Type { MNA, IDA };
		enum class Domain { DP, EMT };

		/// Solve system A * x = z for x and current time
		virtual Real step(Real time, bool blocking = true) = 0;
		/// Log results
		virtual void log(Real time) = 0;
		///
		virtual void addInterface(Interface* eint) { }
		///
		void addSystemTopology(SystemTopology system) { }
		///
		virtual void setSwitchTime(Real switchTime, Int systemIndex) { }
	};
}
