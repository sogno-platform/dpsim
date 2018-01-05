/** Pi Line
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

#include "../Components.h"

namespace DPsim {
namespace Component {
namespace DP {

	class PiLine : public Component::Base {

	protected:
		///resistance in [ohms]
		Real mResistance;

		///Conductance of the line in [S]
		Real mConductance;

		///Capacitance of the line in [F]
		Real mCapacitance;

		///Inductance of the line in [H]
		Real mInductance;

		/// Real part of voltage at node 1 [V]
		Real mVoltageAtNode1Re;
		/// Imaginary part of voltage at node 1 [V]
		Real mVoltageAtNode1Im;
		/// Real part of voltage at node 2 [V]
		Real mVoltageAtNode2Re;
		/// Imaginary part of voltage at node 2 [V]
		Real mVoltageAtNode2Im;
		/// Real part of voltage across the inductor [V]
		Real mDeltaVre;
		/// Imaginary part of voltage across the inductor [V]
		Real mDeltaVim;

		/// Real part of inductor current [A]
		Real mCurrIndRe;
		/// Imaginary part of inductor current [A]
		Real mCurrIndIm;
		/// Real part of capacitor1 current [A]
		Real mCurrCapRe1;
		/// Imaginary part of capacitor1 current [A]
		Real mCurrCapIm1;
		/// Real part of capacitor2 current [A]
		Real mCurrCapRe2;
		/// Imaginary part of capacitor2 current [A]
		Real mCurrCapIm2;

		/// Real part of inductor equivalent current source [A]
		Real mCurEqIndRe;
		/// Imaginary part of inductor equivalent current source [A]
		Real mCurEqIndIm;
		/// Real part of capacitor1 equivalent current source [A]
		Real mCurEqCapRe1;
		/// Imaginary part of capacitor1 equivalent current source [A]
		Real mCurEqCapIm1;
		/// Real part of capacitor2 equivalent current source [A]
		Real mCurEqCapRe2;
		/// Imaginary part of capacitor2 equivalent current source [A]
		Real mCurEqCapIm2;

		/// Real part of inductor equivalent conductance [S]
		Real mGlr;
		/// Imaginary part of inductor equivalent conductance [S]
		Real mGli;
		/// Real part of capacitor equivalent conductance [S]
		Real mGcr;
		/// Imaginary part of capacitor equivalent conductance [S]
		Real mGci;

		/// Auxiliar variable
		Real mPrevCurFacRe;
		/// Auxiliar variable
		Real mPrevCurFacIm;

	public:
		PiLine() { };

		/// Define line name, conected nodes, resistance, inductance and capacitance
		PiLine(String name, Int node1, Int node2, Int node3, Real resistance, Real inductance, Real capacitance);

		/// Initialize voltages and currents values
		void init(Real om, Real dt);

		/// Stamps resistances to the conductance matrix
		void applySystemMatrixStamp(SystemModel& system);

		/// does nothing
		void applyRightSideVectorStamp(SystemModel& system) { }

		/// calculates the value of the DC equivalent current sources for one time step and apply matrix stamp to the current vector
		void step(SystemModel& system, Real time);

		/// Recalculates voltages and currents based on the simulation results of one time step
		void postStep(SystemModel& system);
	};
}
}
}
