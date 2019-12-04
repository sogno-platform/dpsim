/**
 *
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

#include <cps/Config.h>
#include <cps/Terminal.h>
#include <cps/Node.h>
#include <cps/Logger.h>
#include <cps/Definitions.h>
#include <cps/MathUtils.h>
#include <cps/PtrFactory.h>
#include <cps/AttributeList.h>

namespace CPS {
	/// Common base class of all Component templates.
	class PFSolverInterfaceBranch {
	public:
		/// Stamp admittance matrix of the system
		virtual void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) = 0;
    };
}
