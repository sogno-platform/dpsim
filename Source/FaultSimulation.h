/** Simulation with a configurable fault
 *
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

#include "Simulation.h"

using namespace CPS;

namespace DPsim {

	class FaultSimulation : public Simulation {

	protected:
		bool FirstTime = true;
		bool ClearingFault = false;
		bool aCleared = false;
		bool bCleared = false;
		bool cCleared = false;
		Int NumClearedPhases = 0;

		/// Fault Current phase a
		Real mIfa;
		/// Fault Current phase b
		Real mIfb;
		/// Fault Current phase c
		Real mIfc;

		/// Fault Current phase a last time step
		Real mIfa_hist;
		/// Fault Current phase b
		Real mIfb_hist;
		/// Fault Current phase c
		Real mIfc_hist;

		/// Fault Current phase a
		Real mIShifta;
		/// Fault Current phase b
		Real mIShiftb;
		/// Fault Current phase c
		Real mIShiftc;

		/// Fault Current phase a last time step
		Real mIShifta_hist;
		/// Fault Current phase b
		Real mIShiftb_hist;
		/// Fault Current phase c
		Real mIShiftc_hist;

	public:
		// Explicity inheritance of parent constructor
		using Simulation::Simulation;

		Int step(bool blocking = false);

		void clearFault(Int Node1, Int Node2, Int Node3);
	};

}
