/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MathUtils.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/SimTerminal.h>

namespace CPS {
/// Common base class of all Component templates.
class PFSolverInterfaceBranch {
public:
  /// Stamp admittance matrix of the system
  virtual void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow &Y) = 0;
};
} // namespace CPS
