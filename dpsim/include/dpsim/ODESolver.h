/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim/Solver.h>

#include <dpsim-models/Solver/ODEInterface.h>

#include <arkode/arkode.h> // prototypes for ARKode fcts., consts. and includes sundials_types.h
#include <arkode/arkode_direct.h>      // access to ARKDls interface
#include <nvector/nvector_serial.h>    // access to serial N_Vector
#include <sunlinsol/sunlinsol_dense.h> // access to dense SUNLinearSolver
#include <sunmatrix/sunmatrix_dense.h> // access to dense SUNMatrix

//using namespace CPS; // led to problems

namespace DPsim {
/// Solver class for ODE (Ordinary Differential Equation) systems
class ODESolver : public Solver {
protected:
  /// Component to simulate, possible specialized component needed
  CPS::ODEInterface::Ptr mComponent;

  /// Number of differential Variables (states)
  Int mProbDim;

  // ###ARKode-specific variables ###
  /// Memory block allocated by ARKode
  void *mArkode_mem{nullptr};
  /// State vector
  N_Vector mStates{nullptr};

  //for implicit solve:
  /// Indicates whether the ODE shall be solved using an implicit scheme
  bool mImplicitIntegration;
  /// Empty matrix for linear solve in each Newton step while solving the Jacobian Matrix
  SUNMatrix A{nullptr};
  /// Empty linear solver object
  SUNLinearSolver LS{nullptr};

  /// Constant time step
  Real mTimestep;

  // Same tolerance for each component regardless of system characteristics
  /// Relative tolerance
  realtype reltol = RCONST(1.0e-6);
  /// Scalar absolute tolerance
  realtype abstol = RCONST(1.0e-10);

  /// Reusable error-checking flag
  int mFlag{0};

  // Similar to DAE-Solver
  CPS::ODEInterface::StSpFn mStSpFunction;
  CPS::ODEInterface::JacFn mJacFunction;

  /// Use wrappers similar to DAE_Solver
  static int StateSpaceWrapper(realtype t, N_Vector y, N_Vector ydot,
                               void *user_data);
  int StateSpace(realtype t, N_Vector y, N_Vector ydot);

  static int JacobianWrapper(realtype t, N_Vector y, N_Vector fy, SUNMatrix J,
                             void *user_data, N_Vector tmp1, N_Vector tmp2,
                             N_Vector tmp3);
  int Jacobian(realtype t, N_Vector y, N_Vector fy, SUNMatrix J, N_Vector tmp1,
               N_Vector tmp2, N_Vector tmp3);
  /// ARKode- standard error detection function; in DAE-solver not detection function is used -> for efficiency purposes?
  int check_flag(void *flagvalue, const std::string &funcname, int opt);

public:
  /// Create solve object with corresponding component and information on the integration type
  ODESolver(String name, const CPS::ODEInterface::Ptr &comp,
            bool implicit_integration, Real timestep);
  /// Deallocate all memory
  ~ODESolver();

  class SolveTask : public CPS::Task {
  public:
    SolveTask(ODESolver &solver)
        : Task(solver.mName + ".Solve"), mSolver(solver) {
      mAttributeDependencies.push_back(solver.mComponent->mOdePreState);
      mModifiedAttributes.push_back(solver.mComponent->mOdePostState);
    }

    void execute(Real time, Int timeStepCount);

  private:
    ODESolver &mSolver;
  };

  virtual CPS::Task::List getTasks() {
    return CPS::Task::List{std::make_shared<SolveTask>(*this)};
  }

  /// Initialize ARKode-solve_environment
  void initialize();
  /// Solve system for the current time
  Real step(Real initial_time);
};
} // namespace DPsim
