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
