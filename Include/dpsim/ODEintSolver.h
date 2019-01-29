/** DAE Solver
 * 
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/Solver.h>
#include <cps/SystemTopology.h>
#include <cps/Logger.h>
#include <cps/Solver/ODEintInterface.h>

#include <boost/numeric/odeint/stepper/runge_kutta4.hpp> //ODEInt Runge-Kutta stepper
#include <boost/numeric/odeint/integrate/integrate_const.hpp>//ODEInt Integrator with constant time step

namespace DPsim {
    /// Solver class which uses ODE systems
    class ODEintSolver : public Solver {

    protected:
        /// Constant time step
        Real mTimestep;
        ///Problem Size
        int ProbDim;
        /// Stepper needed by ODEint
        boost::numeric::odeint::runge_kutta4<std::vector<Real>> stepper;
        /// Current solution vector
        std::vector<Real> curSolution;
        /// Vector containing the solution at every timestep
        std::vector<std::vector<Real>> solution;
        /// Vector containing all timesteps
        std::vector<Real> times;
        ///ODE of Component
        std::vector<CPS::ODEintInterface::stateFnc> system;

    private:
        ///static pointer to current object; only one instance currently allowed
        inline static ODEintSolver *self = nullptr;
        /// State space of the System and corresponding static wrapper
        static void StateSpaceWrapper(  std::vector<double> y, std::vector<double> ydot, double t);


    public:
        /// Create solve object with given parameters
        ODEintSolver(String name,CPS::ODEintInterface::Ptr comp, Real dt, Real t0);

        /// Solve system for the current time
        Real step(Real time);

        /// Deallocate all memory
        ~ODEintSolver();
    };

}

