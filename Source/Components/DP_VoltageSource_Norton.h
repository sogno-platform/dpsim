/** Real voltage source
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "Base.h"
#include "Base_ControllableSource.h"
#include "Base_ExportableCurrent.h"

namespace DPsim {
namespace Components {
namespace DP {

	/// Real Voltage source:
	/// The real voltage source is a voltage source in series with a resistance, which is transformed to a current source with
	/// a parallel resistance using the Norton equivalent
	class VoltageSourceNorton : public Base, public ControllableSourceBase, public ExportableCurrentBase, public SharedFactory<VoltageSourceNorton> {
	protected:
		/// Voltage [V]
		Complex mVoltage;
		//  ### Real Voltage source parameters ###
		/// Resistance [ohm]
		Real mResistance;
		/// conductance [S]
		Real mConductance;
		/// Equivalent current source [A]
		Complex mCurrent;
	public:
		/// define voltage source paramenters
		VoltageSourceNorton(String name, Int node1, Int node2, Complex voltage, Real resistance,
			Logger::Level loglevel = Logger::Level::NONE);

		VoltageSourceNorton(String name, Int node1, Int node2, Real voltageAbs, Real volagePhase, Real resistance,
			Logger::Level loglevel = Logger::Level::NONE);

		/// Stamps voltage source resistance to the conductance matrix
		void applySystemMatrixStamp(SystemModel& system);

		/// Stamps equivalent current source to the current vector
		void applyRightSideVectorStamp(SystemModel& system);

		/// Stamps equivalent current source to the current vector
		void step(SystemModel& system, Real time);

		void setSourceValue(Complex voltage) { mVoltage = voltage; }

		Complex getCurrent(const SystemModel& system);
	};
}
}
}
