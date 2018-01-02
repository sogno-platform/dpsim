/** Ideal voltage source EMT
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

#include "IdealVoltageSourceEMT.h"

using namespace DPsim;

IdealVoltageSourceEMT::IdealVoltageSourceEMT(String name, Int src, Int dest, Real voltage) : BaseComponent(name, src, dest) {
	mVoltage = voltage;
	mNumVirtualNodes = 1;
	mVirtualNodes = { 0 };
	attrMap["voltage"] = { AttrReal, &this->mVoltage };
}

void IdealVoltageSourceEMT::applySystemMatrixStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		system.addRealToSystemMatrix(mNode1, mVirtualNodes[0], -1);
		system.addRealToSystemMatrix(mVirtualNodes[0], mNode1, -1);
	}

	if (mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode2, mVirtualNodes[0], 1);
		system.addRealToSystemMatrix(mVirtualNodes[0], mNode2, 1);
	}
}

void IdealVoltageSourceEMT::applyRightSideVectorStamp(SystemModel& system) {
	system.addRealToRightSideVector(mVirtualNodes[0], mVoltage);
}

void IdealVoltageSourceEMT::step(SystemModel& system, Real time) {
	system.addRealToRightSideVector(mVirtualNodes[0], mVoltage);
}

Complex IdealVoltageSourceEMT::getCurrent(SystemModel& system) {
	Complex actualcurrent = Complex(system.getRealFromLeftSideVector(mVirtualNodes[0]), 0);
	return Complex(system.getRealFromLeftSideVector(mVirtualNodes[0]),0);
}
