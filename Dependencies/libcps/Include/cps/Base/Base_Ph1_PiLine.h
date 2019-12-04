/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
