/**
 *
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

#include<vector>
#include <cps/Definitions.h>

namespace CPS {
	class DAEInterface {
	public:
		typedef std::shared_ptr<DAEInterface> Ptr;
		typedef std::vector<Ptr> List;
		
		using ResFn = std::function<void(double, const double *, const double *, double *, std::vector<int>&)>;

		// #### DAE Section ####
		///Residual Function for DAE Solver
		virtual void daeResidual(double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) = 0;
		///Voltage Getter for Components
		virtual Complex daeInitialize()=0;
	};
}
