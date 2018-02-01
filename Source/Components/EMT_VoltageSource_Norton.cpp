/** Real voltage source (EMT)
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

#include "EMT_VoltageSource_Norton.h"

using namespace DPsim;

Components::EMT::VoltageSourceNorton::VoltageSourceNorton(String name, Int node1, Int node2,
	Real voltageAmp, Real voltagePhase, Real resistance, Logger::Level loglevel)
	: Component(name, node1, node2, loglevel) {
	mVoltageAmp = voltageAmp;
	mVoltagePhase = voltagePhase;
	mResistance = resistance;
	mConductance = 1. / mResistance;
	mVoltage = mVoltageAmp * cos(mVoltagePhase);
	mCurrent = mVoltage / mResistance;
}

Components::EMT::VoltageSourceNorton::VoltageSourceNorton(String name, Int node1, Int node2,
	Complex voltage, Real resistance, Logger::Level loglevel)
	: VoltageSourceNorton(name, node1, node2, std::abs(voltage), std::arg(voltage), resistance, loglevel) {
}

void Components::EMT::VoltageSourceNorton::applySystemMatrixStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode1, mConductance);
	}
	if (mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode2, mNode2, mConductance);
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode2, -mConductance);
		system.addRealToSystemMatrix(mNode2, mNode1, -mConductance);
	}
}

void Components::EMT::VoltageSourceNorton::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}

void Components::EMT::VoltageSourceNorton::step(SystemModel& system, Real time) {
	mVoltage = mVoltageAmp * cos(mVoltagePhase + system.getOmega() * time);
	mCurrent = mVoltage / mResistance;

	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}
