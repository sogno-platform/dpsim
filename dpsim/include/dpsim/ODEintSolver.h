/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <iostream>
#include <list>
#include <vector>

#include <dpsim-models/Solver/ODEintInterface.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim/Solver.h>

#include <boost/numeric/odeint/integrate/integrate_const.hpp> //ODEInt Integrator with constant time step
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp> //ODEInt Runge-Kutta stepper

namespace DPsim {
/// Solver class which uses ODE systems
class ODEintSolver : public Solver {

protected:
  /// Pointer to current Component
  CPS::ODEintInterface::Ptr mComponent;
  /// Constant time step
  Real mTimestep;
  ///Problem Size
  int ProbDim;
  /// Stepper needed by ODEint
  boost::numeric::odeint::runge_kutta4<std::vector<Real>> stepper;
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
  static void StateSpaceWrapper(const std::vector<double> &y,
                                std::vector<double> ydot, double t);

public:
  /// Current solution vector
  std::vector<Real> curSolution;
  /// Create solve object with given parameters
  ODEintSolver(String name, CPS::ODEintInterface::Ptr comp, Real dt, Real t0);

  /// Solve system for the current time
  Real step(Real time);

  /// Deallocate all memory
  ~ODEintSolver();
};

} // namespace DPsim
