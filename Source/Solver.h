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
#include "cps/Interfaces/ExternalInterface.h"
#include "cps/SystemTopology.h"

using namespace CPS;

namespace DPsim {
	/// Ground node
	const Int GND = -1;
	/// Holds switching time and which system should be activated.
	struct SwitchConfiguration {
		Real switchTime;
		UInt systemIndex;
	};

	class Solver {
	public:
		enum class Type { MNA, IDA };
		enum class Domain { DP, EMT };

		/// Run simulation until total time is elapsed.
		virtual void run() = 0;
		/// Run simulation for \p duration seconds.
		virtual void run(double duration) { }
		///
		virtual void addExternalInterface(ExternalInterface* eint) { }
		///
		void addSystemTopology(SystemTopology system) { }
		///
		virtual void setSwitchTime(Real switchTime, Int systemIndex) { }
	};
}
