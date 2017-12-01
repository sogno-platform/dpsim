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

#include "BaseComponent.h"
#include "InductorDP.h"
#include "ResistorDP.h"

namespace DPsim {
	// TODO currently modeled as an impedance, which obviously doesn't have a constant power characteristic
	class PQLoadDP : public BaseComponent {
	protected:
		/// Active power [Watt]
		Real mActivePower;
		/// Reactive power [VAr]
		Real mReactivePower;
		/// Voltage [V]
		Real mSvVoltage;
		/// Resistance [Ohm]
		Real mResistance;
		/// Conductance [S]
		Real mConductance;
		/// Reactance [Ohm]
		Real mReactance;
		/// Inductance [H]
		Real mInductance;
		
		shared_ptr<InductorDP> inductor;
		shared_ptr<ResistorDP> resistor;
	public:
		PQLoadDP(String name, Int node, Real activePower, Real reactivePower, Real volt, Real angle);

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
