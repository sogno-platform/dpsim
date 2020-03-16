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

#include <vector>
#include <cps/AttributeList.h>
#include <cps/Definitions.h>

namespace CPS {
	class ODEInterface : virtual public AttributeList {
	public:
		typedef std::shared_ptr<ODEInterface> Ptr;
		//typedef std::vector<Ptr> List;

		/// Use this to pass the individual components StateSpace implementation to the ODESolver class
		using StSpFn = std::function<void(double, const double *, double *)>;

		using JacFn = std::function<void(double, const double *, double *, double *,
		                                 double *, double *, double *)>;

		// #### ODE Section ####
		/// State Space Equation System for ODE Solver
		virtual void odeStateSpace(double t, const double y[], double ydot[]) = 0;

		/// Jacobian Matrix (of State Space System) needed for implicit solve
		virtual void odeJacobian(double t, const double y[], double fy[], double J[],
		                         double tmp1[], double tmp2[], double tmp3[]) = 0;

	protected:
		ODEInterface() {
			addAttribute<Matrix>("ode_pre_state", &mOdePreState, Flags::read);
			addAttribute<Matrix>("ode_post_state", &mOdePostState, Flags::read | Flags::write);
		}

		Matrix mOdePreState, mOdePostState;
	};
}
