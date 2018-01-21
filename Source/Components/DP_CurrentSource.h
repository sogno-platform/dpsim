/** Current source
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

#include "Component.h"
#include "Base_ControllableSource.h"
#include "Base_ExportableCurrent.h"

namespace DPsim {
namespace Components {
namespace DP {

	class CurrentSource : public Component, public ControllableSourceBase, public ExportableCurrentBase, public SharedFactory<CurrentSource> {
	private:
		Complex mCurrent;
	public:
		CurrentSource(String name, Int node1, Int node2, Complex current);
		CurrentSource(String name, Int node1, Int node2, Real currentAbs, Real currentPhase);

		void initialize(SystemModel& system) { }
		void applySystemMatrixStamp(SystemModel& system) { }
		void applyRightSideVectorStamp(SystemModel& system);
		void step(SystemModel& system, Real time);
		void postStep(SystemModel& system) { }
		void setSourceValue(Complex value) { mCurrent = value; }

		Complex getCurrent(const SystemModel& system);
	};
}
}
}
