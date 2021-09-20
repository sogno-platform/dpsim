/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Config.h>
#include <cps/SimTerminal.h>
#include <cps/SimNode.h>
#include <cps/Logger.h>
#include <cps/Definitions.h>
#include <cps/MathUtils.h>
#include <cps/PtrFactory.h>
#include <cps/AttributeList.h>

namespace CPS {
	/// Common base class of all Component templates.
	class PFSolverInterfaceBranch {
	public:
		/// Stamp admittance matrix of the system
		virtual void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow & Y) = 0;
    };
}
