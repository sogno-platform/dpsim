/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	/// \brief MNA interface to be used by switching devices.
	class MNASwitchInterface : public MNAInterface {
	public:
		typedef std::shared_ptr<MNASwitchInterface> Ptr;
		typedef std::vector<Ptr> List;

		// #### MNA section ####
		/// Open switch
		virtual void mnaOpen() { }
		/// Close switch
		virtual void mnaClose() { }
		/// Check if switch is closed
		virtual Bool mnaIsClosed() = 0;
		/// Close switch if true and open switch if false
		virtual void mnaSetClosed(Bool value) { }
		/// Stamps system matrix considering the defined switch position
		virtual void mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed) { }
	};
}