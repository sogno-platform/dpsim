/**
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017-2019, Institute for Automation of Complex Power Systems, EONERC
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
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	class MNATearInterface : public MNAInterface {
	public:
		// Returns a list of additional components connected to ground that
		// need to be considered for the original systems.
		virtual MNAInterface::List mnaTearGroundComponents() { return MNAInterface::List(); }
		// Initialize the internal state of the component
		virtual void mnaTearInitialize(Real omega, Real timeStep) {}
		// Apply the stamp to the impedance matrix of the removed network
		virtual void mnaTearApplyMatrixStamp(Matrix& tearMatrix) = 0;
		// TODO: if we're consequent, these should be implemented as tasks
		// Apply the stamp to the vector of additional voltages in the removed network
		virtual void mnaTearApplyVoltageStamp(Matrix& currentVector) {}
		// Update the internal state based on the solution of the complete system
		virtual void mnaTearPostStep(Complex voltage, Complex current) {}

		void mnaTearSetIdx(UInt compIdx) { mTearIdx = compIdx; }

	protected:
		UInt mTearIdx;
	};
}
