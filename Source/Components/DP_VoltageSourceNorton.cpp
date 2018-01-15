/** Real voltage source
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

#include "DP_VoltageSourceNorton.h"

using namespace DPsim;

Components::DP::VoltageSourceNorton::VoltageSourceNorton(String name, Int node1, Int node2, Complex voltage, Real resistance)
	: Base(name, node1, node2) {
	mVoltage = voltage;
	mResistance = resistance;
	attrMap["voltage"] = { Attribute::Complex, &mVoltage };
	attrMap["resistance"] = { Attribute::Real, &mResistance };
}

Components::DP::VoltageSourceNorton::VoltageSourceNorton(String name, Int node1, Int node2, Real voltageAbs, Real voltagePhase, Real resistance)
	: Base(name, node1, node2) {
	mVoltage = MathLibrary::polar(voltageAbs, voltagePhase);
	mResistance = resistance;
	attrMap["voltage"]    = { Attribute::Complex, &mVoltage };
	attrMap["resistance"] = { Attribute::Real, &mResistance };
}

void Components::DP::VoltageSourceNorton::applySystemMatrixStamp(SystemModel& system) {
	mConductance = 1. / mResistance;
	mCurrent = mVoltage / mResistance;

	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, Complex(mConductance, 0));
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, Complex(mConductance, 0));
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, Complex(-mConductance, 0));
		system.addCompToSystemMatrix(mNode2, mNode1, Complex(-mConductance, 0));
	}
}

void Components::DP::VoltageSourceNorton::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrent);
	}
}

void Components::DP::VoltageSourceNorton::step(SystemModel& system, Real time) {
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrent);
	}
}

Complex Components::DP::VoltageSourceNorton::getCurrent(SystemModel& system) {
	Complex retCurrent;
	if (mNode1 >= 0) {
		retCurrent += system.getCompFromLeftSideVector(mNode1) * mConductance;
	}
	if (mNode2 >= 0) {
		retCurrent -= system.getCompFromLeftSideVector(mNode2) * mConductance;
	}
	return retCurrent;
}
