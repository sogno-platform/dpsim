/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
/// MNA interface to be used by nonlinear elements that require recomputing
//	of the system matrix
class MNANonlinearVariableCompInterface
    : virtual public MNAVariableCompInterface {
public:
  typedef std::shared_ptr<MNANonlinearVariableCompInterface> NonlinearPtr;
  typedef std::vector<NonlinearPtr> NonlinearList;

  const Attribute<Matrix>::Ptr mNonlinearFunctionStamp;
  std::vector<std::pair<UInt, UInt>> mNonlinearVariableSystemMatrixEntries;

  /// Returns value of the component's defining voltage/current equation
  //	based on current circuit quantities
  virtual void calculateNonlinearFunctionResult() = 0;
  virtual void iterationUpdate(const Matrix &leftVector) = 0;

protected:
  MNANonlinearVariableCompInterface()
      : mNonlinearFunctionStamp(AttributeStatic<Matrix>::make()){};
};
} // namespace CPS
