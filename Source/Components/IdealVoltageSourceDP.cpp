/** Ideal voltage source
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

#include "IdealVoltageSourceDP.h"

using namespace DPsim;

IdealVoltageSource::IdealVoltageSource(String name, Int src, Int dest, Complex voltage) : BaseComponent(name, src, dest) {
	mVoltage = voltage;
	mNumVirtualNodes = 1;
	mVirtualNodes = { 0 };
	attrMap["voltage"] = {AttrComplex, &this->mVoltage};
}

void IdealVoltageSource::applySystemMatrixStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode1, 1, 0);
		system.setCompSystemMatrixElement(mNode1, mVirtualNodes[0], 1, 0);
	}

	if (mNode2 >= 0) {
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode2, -1, 0);
		system.setCompSystemMatrixElement(mNode2, mVirtualNodes[0], -1, 0);
	}
}

void IdealVoltageSource::applyRightSideVectorStamp(SystemModel& system) {
	system.addCompToRightSideVector(mVirtualNodes[0], mVoltage.real(), mVoltage.imag());
}

void IdealVoltageSource::step(SystemModel& system, Real time) {
	system.addCompToRightSideVector(mVirtualNodes[0], mVoltage.real(), mVoltage.imag());
}

Complex IdealVoltageSource::getCurrent(SystemModel& system) {
	return Complex(system.getRealFromLeftSideVector(mVirtualNodes[0]), system.getRealFromLeftSideVector(mVirtualNodes[0] + system.getCompOffset()));
}
