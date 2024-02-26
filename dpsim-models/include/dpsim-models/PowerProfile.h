/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once
#include <dpsim-models/Definitions.h>

namespace CPS {
struct PQData {
  Real p;
  Real q;
};

struct PowerProfile {
  std::map<Real, PQData> pqData;
  std::map<Real, Real> weightingFactors;
};
} // namespace CPS
