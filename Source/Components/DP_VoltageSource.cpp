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

#include "DP_VoltageSource.h"

using namespace DPsim;

Components::DP::VoltageSource::VoltageSource(String name, Int node1, Int node2, Complex voltage, Logger::Level loglevel)
	: Base(name, node1, node2, loglevel) {
	mVoltage = voltage;
	mNumVirtualNodes = 1;
	mVirtualNodes = { 0 };
	attrMap["voltage"] = { Attribute::Complex, &mVoltage };
	mLog.Log(Logger::Level::DEBUG) << "Create VoltageSource " << name << " at " << mNode1 << "," << mNode2 << std::endl;
}

Components::DP::VoltageSource::VoltageSource(String name, Int node1, Int node2, Real voltageAbs, Real voltagePhase,
	Logger::Level loglevel)
	: Base(name, node1, node2, loglevel) {
	mVoltage = MathLibrary::polar(voltageAbs, voltagePhase);
	mNumVirtualNodes = 1;
	mVirtualNodes = { 0 };
	attrMap["voltage"] = { Attribute::Complex, &mVoltage };
	mLog.Log(Logger::Level::DEBUG) << "Create VoltageSource " << name << " at " << mNode1 << "," << mNode2 << std::endl;
}

void Components::DP::VoltageSource::applySystemMatrixStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << Complex(-1, 0) << " to " << mNode1 << "," << mVirtualNodes[0] << std::endl;
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode1, Complex(-1, 0));
		mLog.Log(Logger::Level::DEBUG) << "Add " << Complex(-1, 0) << " to " << mVirtualNodes[0] << "," << mNode1 << std::endl;
		system.setCompSystemMatrixElement(mNode1, mVirtualNodes[0], Complex(-1, 0));
	}

	if (mNode2 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << Complex(1, 0) << " to " << mVirtualNodes[0] << "," << mNode2 << std::endl;
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode2, Complex(1, 0));
		mLog.Log(Logger::Level::DEBUG) << "Add " << Complex(1, 0) << " to " << mNode2 << "," << mVirtualNodes[0] << std::endl;		
		system.setCompSystemMatrixElement(mNode2, mVirtualNodes[0], Complex(1, 0));
	}
}

void Components::DP::VoltageSource::applyRightSideVectorStamp(SystemModel& system) {
	mLog.Log(Logger::Level::DEBUG) << "Add " << mVoltage << " to right side " << mVirtualNodes[0] << std::endl;
	system.addCompToRightSideVector(mVirtualNodes[0], mVoltage);
}

void Components::DP::VoltageSource::step(SystemModel& system, Real time) {
	system.addCompToRightSideVector(mVirtualNodes[0], mVoltage);
}

Complex Components::DP::VoltageSource::getCurrent(SystemModel& system) {
	return Complex(system.getRealFromLeftSideVector(mVirtualNodes[0]), system.getRealFromLeftSideVector(mVirtualNodes[0] + system.getCompOffset()));
}

void Components::DP::VoltageSource::setSourceValue(Complex voltage) {
	mVoltage = voltage;
}
