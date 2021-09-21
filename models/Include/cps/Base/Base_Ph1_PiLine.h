/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>

namespace CPS {
namespace Base {
namespace Ph1 {
	class PiLine {
	protected:
		/// Resistance along the line [ohms]
		Real mSeriesRes;
		/// Conductance along the line [S]
		Real mSeriesCond;
		/// Inductance along the line [H]
		Real mSeriesInd;
		/// Resistance in parallel to the line [ohms]
		Real mParallelRes;
		/// Conductance in parallel to the line [S]
		Real mParallelCond;
		/// Capacitance in parallel to the line [F]
		Real mParallelCap;
	public:
		///
		void setParameters(Real seriesResistance, Real seriesInductance,
			Real parallelCapacitance = 0, Real parallelConductance = 0) {
			mSeriesRes = seriesResistance;
			mSeriesCond = 1. / mSeriesRes;
			mSeriesInd = seriesInductance;
			mParallelCond = parallelConductance;
			mParallelRes = 1. / mParallelCond;
			mParallelCap = parallelCapacitance;
		}
	};
}
}
}
