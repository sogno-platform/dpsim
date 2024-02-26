/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Definitions.h>
#include <vector>

namespace CPS {
class ODEInterface {
public:
  using Ptr = std::shared_ptr<ODEInterface>;
  //typedef std::vector<Ptr> List;

  const CPS::AttributeList::Ptr mAttributeList;

  const CPS::Attribute<Matrix>::Ptr mOdePreState;
  const CPS::Attribute<Matrix>::Ptr mOdePostState;

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
  explicit ODEInterface(AttributeList::Ptr attrList)
      : mAttributeList(attrList),
        mOdePreState(attrList->create<Matrix>("ode_pre_state")),
        mOdePostState(attrList->create<Matrix>("ode_post_state")) {}
};
} // namespace CPS
