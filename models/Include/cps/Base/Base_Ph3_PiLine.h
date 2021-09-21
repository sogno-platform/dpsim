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
namespace Ph3 {
	class PiLine {
	protected:
		/// Resistance along the line [ohms]
		Matrix mSeriesRes;
		/// Conductance along the line [S]
		Matrix mSeriesCond;
		/// Inductance along the line [H]
		Matrix mSeriesInd;
		/// Resistance in parallel to the line [ohms]
		Matrix mParallelRes;
		/// Conductance in parallel to the line [S]
		Matrix mParallelCond;
		/// Capacitance in parallel to the line [F]
		Matrix mParallelCap;
	public:
		///
		void setParameters(Matrix seriesResistance, Matrix seriesInductance,
			Matrix parallelCapacitance = Matrix::Zero(3,3), Matrix parallelConductance = Matrix::Zero(3,3)) {
			mSeriesRes = seriesResistance;
			mSeriesCond = mSeriesRes.inverse();
			mSeriesInd = seriesInductance;
			mParallelCond = parallelConductance;
			mParallelRes = mParallelCond.inverse();
			mParallelCap = parallelCapacitance;
		}
	};
}
}
}
