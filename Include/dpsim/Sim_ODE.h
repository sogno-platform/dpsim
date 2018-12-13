/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#include <dpsim/Simulation.h>
#include <dpsim/ODESolver.h>

namespace DPsim {
  /// @brief enhances the regular Simulation Class with multiple ODE-Solver units
  class Sim_ODE: public Simulation{
  protected:
      /// Stores all ODE-Solver objects corresponding to a certain component
      std::vector<std::shared_ptr<ODESolver> >mODESolverList;
  public:
    /// Constructor with pre-initialized components
    Sim_ODE(String name, CPS::SystemTopology system,
			Real timeStep, Real finalTime, std::vector<std::shared_ptr<ODESolver> > ODESolverList,
			CPS::Domain domain = CPS::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO);

    /// Constructor for non-pre initialized components
    Sim_ODE(String name, CPS::SystemTopology system,
			Real timeStep, Real finalTime,
			CPS::Domain domain = CPS::Domain::DP,
			Solver::Type solverType = Solver::Type::MNA,
			CPS::Logger::Level logLevel = CPS::Logger::Level::INFO);

    /// Implementation of step-function
    Real step();

    /// Add initialized components
    void addSolver(std::shared_ptr<ODESolver> solver);
  };
}
