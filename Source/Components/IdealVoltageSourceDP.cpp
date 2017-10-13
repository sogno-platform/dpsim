/** Ideal voltage source
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

#include "IdealVoltageSourceDP.h"

using namespace DPsim;

IdealVoltageSource::IdealVoltageSource(std::string name, Int src, Int dest, Complex voltage, Int num) : BaseComponent(name, src, dest) {
	this->number = num;
	this->mVoltage = voltage;
	attrMap["voltage"] = {AttrComplex, &this->mVoltage};
}

void IdealVoltageSource::applySystemMatrixStamp(SystemModel& system) {
	number = system.getNumIdealVS() - number + 1;
	if (mNode1 >= 0) {
		system.setSystemMatrixElement(system.getCompOffset() - number, mNode1, 1);
		system.setSystemMatrixElement(mNode1, system.getCompOffset() - number, 1);
		system.setSystemMatrixElement(2* system.getCompOffset() - number, mNode1 + system.getCompOffset(), 1);
		system.setSystemMatrixElement(mNode1 + system.getCompOffset(), 2 * system.getCompOffset() - number, 1);
	}

	if (mNode2 >= 0) {
		system.setSystemMatrixElement(system.getCompOffset() - number, mNode2, -1);
		system.setSystemMatrixElement(mNode2, system.getCompOffset() - number, -1);
		system.setSystemMatrixElement(2 * system.getCompOffset() - number, mNode2 + system.getCompOffset(), -1);
		system.setSystemMatrixElement(mNode2 + system.getCompOffset(), 2 * system.getCompOffset() - number, -1);
	}
}

void IdealVoltageSource::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	system.addRealToRightSideVector(system.getCompOffset() - number, mVoltage.real());
	system.addRealToRightSideVector(2 * system.getCompOffset() - number, mVoltage.imag());
}

void IdealVoltageSource::step(SystemModel& system, Real time) {
	// Apply matrix stamp for equivalent current source
	system.addRealToRightSideVector(system.getCompOffset() - number, mVoltage.real());
	system.addRealToRightSideVector(2 * system.getCompOffset() - number, mVoltage.imag());
}

Complex IdealVoltageSource::getCurrent(SystemModel& system) {
	return Complex(system.getRealFromLeftSideVector(system.getCompOffset()-number), system.getRealFromLeftSideVector(2*system.getCompOffset()-number));
}
