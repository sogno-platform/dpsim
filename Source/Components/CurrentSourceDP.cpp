/** Current source
 *
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

#include "CurrentSourceDP.h"

using namespace DPsim;

CurrentSource::CurrentSource(std::string name, int src, int dest, Complex current) : BaseComponent(name, src, dest) {
	this->mCurrent = current;
	attrMap["current"] = {AttrComplex, &this->mCurrent};
};

void CurrentSource::applyRightSideVectorStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrent.real(), mCurrent.imag());
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrent.real(), mCurrent.imag());
	}
}

void CurrentSource::step(SystemModel& system, Real time) {
	this->applyRightSideVectorStamp(system);
}

Complex CurrentSource::getCurrent(SystemModel &system) {
	return mCurrent;
}
