/** Inductor
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

	/// Inductor model:
	/// The inductor is represented by a DC equivalent circuit which corresponds to one iteration of the trapezoidal integration method.
	/// The equivalent DC circuit is a resistance in paralel with a current source. The resistance is constant for a defined time step and system
	///frequency and the current source changes for each iteration.
	class Inductor : public BaseComponent {
	protected:

		/// Inductance [H]
		Real mInductance;

		/// Real part of the voltage across the inductor [V]
		Real mDeltaVre;
		/// Imaginary part of the voltage across the inductor [V]
		Real mDeltaVim;

		/// Real part of the current trough the inductor [A]
		Real mCurrRe;
		/// Imaginary part of the current trough the inductor [A]
		Real mCurrIm;

		/// Real part of the DC equivalent current source [A]
		Real mCurEqRe;
		/// Imaginary part of the DC equivalent current source [A]
		Real mCurEqIm;

		/// Real part of the DC equivalent conductance [S]
		Real mGlr;
		/// Imaginary part of the DC equivalent conductance [S]
		Real mGli;

		/// Auxiliar variables
		Real mPrevCurFacRe;
		Real mPrevCurFacIm;

	public:
		Inductor() { };

		/// Define inductor name, conected nodes and inductance
		Inductor(std::string name, Int src, Int dest, double inductance);

		/// Initializes variables detalvr, deltavi, currr, curri, cureqr and curreqi
		void init(Real om, Real dt);

		/// Stamps DC equivalent resistance to the conductance matrix
		void applySystemMatrixStamp(SystemModel& system);

		/// Does nothing
		void applyRightSideVectorStamp(SystemModel& system) { }

		/// calculates the value of the DC equivalent current source for one time step and apply matrix stamp to the current vector
		void step(SystemModel& system, Real time);

		/// Recalculates variables detalvr, deltavi, currr and curri based on the simulation results of one time step
		void postStep(SystemModel& system);

		/// Return current from the previous step
		Complex getCurrent(SystemModel& system);
	};
}
