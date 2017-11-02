/** PQ Load
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
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

#include "RxLineDP.h"

namespace DPsim {
	// TODO currently modeled as an impedance, which obviously doesn't have a constant power characteristic
	class PQLoad : public RxLine {
	protected:
		Real mActivePower;
		Real mReactivePower;
		Real mSvVoltage;
	public:
		PQLoad(String name, Int src, Int dest, Real p, Real q, Real volt, Real angle);
		void init(Real om, Real dt);
		void applySystemMatrixStamp(SystemModel&);
	};
}
