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

#include "DP_CurrentSource.h"

using namespace DPsim;

Components::DP::CurrentSource::CurrentSource(String name, Int node1, Int node2, Complex current)
	: Base(name, node1, node2) {
	mCurrent = current;
	attrMap["current"] = { Attribute::Complex, &mCurrent };
}

Components::DP::CurrentSource::CurrentSource(String name, Int node1, Int node2, Real currentAbs, Real currentPhase)
	: Base(name, node1, node2) {
	mCurrent = MathLibrary::polar(currentAbs, currentPhase);
	attrMap["current"] = { Attribute::Complex, &mCurrent };
}

void Components::DP::CurrentSource::applyRightSideVectorStamp(SystemModel& system)
{
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, Complex(-mCurrent.real(), mCurrent.imag()));
	}
}

void Components::DP::CurrentSource::step(SystemModel& system, Real time)
{
	applyRightSideVectorStamp(system);
}

Complex Components::DP::CurrentSource::getCurrent(SystemModel &system)
{
	return mCurrent;
}
