/** Current source
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

#include "EMT_CurrentSource.h"

using namespace DPsim;

Components::EMT::CurrentSource::CurrentSource(String name, Int node1, Int node2, Real currentAmp, Real currentPhase)
	: Component(name, node1, node2) {
	mCurrent = currentAmp * cos(currentPhase);
	attrMap["current"] = { Attribute::Real, &mCurrent };
}

Components::EMT::CurrentSource::CurrentSource(String name, Int node1, Int node2, Complex current)
	: CurrentSource(name, node1, node2, std::abs(current), std::arg(current)) {
}

void Components::EMT::CurrentSource::applyRightSideVectorStamp(SystemModel& system)
{
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}

void Components::EMT::CurrentSource::step(SystemModel& system, Real time)
{
	applyRightSideVectorStamp(system);
}

Real Components::EMT::CurrentSource::getCurrent(const SystemModel &system)
{
	return mCurrent;
}
