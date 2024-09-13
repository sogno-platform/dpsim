/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/DP/DP_Ph3_SynchronGeneratorDQ.h>

#ifdef WITH_SUNDIALS
#include <arkode/arkode.h>             // Prototypes for ARKode fcts., consts
//#include <arkode/arkode_direct.h>      /* access to ARKDls interface				   */
#include <nvector/nvector_serial.h>    // Serial N_Vector types, fcts., macros
#include <sundials/sundials_types.h>   /* def. of type 'sunsunrealtype' */
#include <sunlinsol/sunlinsol_dense.h> /* access to dense SUNLinearSolver		  */
#include <sunmatrix/sunmatrix_dense.h> /* access to dense SUNMatrix						*/

#include <dpsim-models/Solver/ODEInterface.h>
// Needed to print the computed to the console
#if defined(SUNDIALS_EXTENDED_PRECISION)
#define GSYM "Lg"
#define ESYM "Le"
#define FSYM "Lf"
#else
#define GSYM "g"
#define ESYM "e"
#define FSYM "f"
#endif
#endif

namespace CPS {
namespace DP {
namespace Ph3 {
class SynchronGeneratorDQODE : public SynchronGeneratorDQ,
                               public ODEInterface,
                               public SharedFactory<SynchronGeneratorDQODE> {
public:
  SynchronGeneratorDQODE(String uid, String name,
                         Logger::Level loglevel = Logger::Level::off);
  SynchronGeneratorDQODE(String name,
                         Logger::Level loglevel = Logger::Level::off);

  // #### MNA Section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;

  /// Add MNA pre step dependencies
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;

  class ODEPreStep : public Task {
  public:
    ODEPreStep(SynchronGeneratorDQODE &synGen)
        : Task(**synGen.mName + ".ODEPreStep"), mSynGen(synGen) {
      mModifiedAttributes.push_back(synGen.attribute("ode_pre_state"));
      mModifiedAttributes.push_back(synGen.attribute("i_intf"));
    }

    void execute(Real time, Int timeStepCount);

  private:
    SynchronGeneratorDQODE &mSynGen;
  };

protected:
  // #### ODE Section ####

  /// Defines the ODE-System which shall be solved
  void odeStateSpace(double t, const double y[],
                     double ydot[]) override; // ODE-Class approach

  /// Jacobian corresponding to the StateSpace system, needed for implicit solve
  void odeJacobian(double t, const double y[], double fy[], double J[],
                   double tmp1[], double tmp2[], double tmp3[]) override;

  void odePreStep();
  void odePostStep();

  // ### Variables for ODE-Solver interaction ###
  /// Number of differential variables
  int mDim;
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
