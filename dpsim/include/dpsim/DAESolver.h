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

#include <dpsim/Solver.h>

#include <dpsim-models/Logger.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/SystemTopology.h>

#include <ida/ida.h>
#include <ida/ida_direct.h>
#include <nvector/nvector_serial.h>
#include <sundials/sundials_types.h>
#include <sunlinsol/sunlinsol_dense.h>

namespace DPsim {

/// Solver class which uses Differential Algebraic Equation(DAE) systems
class DAESolver : public Solver {
protected:
  // General simulation parameters
  CPS::SystemTopology mSystem;
  /// Offsets vector for adding new equations to the residual vector
  std::vector<Int> mOffsets;
  /// Constant time step
  Real mTimestep;
  /// Number of equations in problem
  Int mNEQ;
  /// Components of the Problem
  CPS::IdentifiedObject::List mComponents;
  /// Nodes of the Problem
  CPS::SimNode<Complex>::List mNodes;

  // Initial time t0
  Real mT0;

  // IDA simulation variables
  /// Memory block allocated by IDA
  void *mem = NULL;
  /// Vector of problem variables
  N_Vector state = NULL;
  /// Derivates of the state vector with respect to time
  N_Vector dstate_dt = NULL;
  /// Time IDA reached while solving
  realtype tret;
  /// Scalar absolute tolerance
  realtype abstol;
  /// Relative tolerance
  realtype rtol;
  /// Template Jacobian Matrix
  SUNMatrix A = NULL;
  /// Linear solver object
  SUNLinearSolver LS = NULL;
  long int interalSteps = 0;
  long int resEval = 0;
  std::vector<CPS::DAEInterface::ResFn> mResidualFunctions;

  /// Residual Function of entire System
  static int residualFunctionWrapper(realtype ttime, N_Vector state,
                                     N_Vector dstate_dt, N_Vector resid,
                                     void *user_data);
  int residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt,
                       N_Vector resid);

public:
  /// Create solve object with given parameters
  DAESolver(String name, const CPS::SystemTopology &system, Real dt, Real mT0);
  /// Deallocate all memory
  ~DAESolver();
  /// Initialize Components & Nodes with inital values
  void initialize() override;

  /// Solve system for the current time
  Real step(Real time);

  CPS::Task::List getTasks();
};
} // namespace DPsim
