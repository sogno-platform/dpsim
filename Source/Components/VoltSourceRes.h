/** Real voltage source
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

#include "BaseComponent.h"

namespace DPsim {

	class VoltSourceRes : public BaseComponent {

		/// Real Voltage source:
		/// The real voltage source is a voltage source in series with a resistance, which is transformed to a current source with
		/// a parallel resistance using the Norton equivalent

	protected:

		//  ### Real Voltage source parameters ###
		/// Complex voltage [V]
		Complex mVoltage;

		/// Resistance [ohm]
		Real mResistance;

		/// conductance [S]
		Real mConductance;

		/// Real part of equivalent current source [A]
		Real mCurrentr;
		/// Imaginary part of equivalent current source [A]
		Real mCurrenti;

	public:
		VoltSourceRes() { ; };

		/// define voltage source paramenters
		VoltSourceRes(std::string name, int src, int dest, Complex voltage, Real resistance);

		void init(Real om, Real dt) { }

		/// Stamps voltage source resistance to the conductance matrix
		void applySystemMatrixStamp(SystemModel& system);

		/// Stamps equivalent current source to the current vector
		void applyRightSideVectorStamp(SystemModel& system);

		/// Stamps equivalent current source to the current vector
		void step(SystemModel& system, Real time);

		void postStep(SystemModel& system) { }

		Complex getCurrent(SystemModel& system);
	};
}
