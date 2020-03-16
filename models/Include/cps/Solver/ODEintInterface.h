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
	class ODEintInterface {
	public:
		typedef std::shared_ptr<ODEintInterface> Ptr;
        using stateFnc = std::function<void(const double *,  double *,  const double )>;

		// #### ODE Section ####
		/// Returns number of differential variables
		virtual int num_states() const=0;
		/// Sets up ODE system in ydot
		virtual void odeint(const double y[], double ydot[], double t) = 0;

		/// Needed for computations which have to be carried out before the numerical approximation step
		virtual void pre_step()=0;
		/// Writes the values from the constructed state vector back into the original fields
		virtual void post_step()=0;
		///Returns Pointer to state Vector of the componente
		virtual double* state_vector() = 0;
		///Writes the computed solution to the component
		virtual void set_state_vector(std::vector<double> y) = 0;
	};
}
