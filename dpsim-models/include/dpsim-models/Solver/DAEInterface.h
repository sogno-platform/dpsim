/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>
#include <vector>

namespace CPS {
class DAEInterface {
public:
  typedef std::shared_ptr<DAEInterface> Ptr;
  typedef std::vector<Ptr> List;

  using ResFn = std::function<void(double, const double *, const double *,
                                   double *, std::vector<int> &)>;

  // #### DAE Section ####
  ///Residual Function for DAE Solver
  virtual void daeResidual(double ttime, const double state[],
                           const double dstate_dt[], double resid[],
                           std::vector<int> &off) = 0;
  ///Voltage Getter for Components
  virtual Complex daeInitialize() = 0;
};
} // namespace CPS
