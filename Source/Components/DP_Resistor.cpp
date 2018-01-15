/** Linear Resistor
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

#include "DP_Resistor.h"

using namespace DPsim;

Components::DP::Resistor::Resistor(String name, Int node1, Int node2, Real resistance,
	Logger::Level logLevel)
	: Base(name, node1, node2, logLevel) {
	mResistance = resistance;
	attrMap["resistance"] = { Attribute::Real, &mResistance };
	mLog.Log(Logger::Level::DEBUG) << "Create Resistor " << name << " at " << mNode1 << "," << mNode2 << std::endl;
}

void Components::DP::Resistor::applySystemMatrixStamp(SystemModel& system)
{
	mConductance = 1.0 / mResistance;

	// Set diagonal entries
	if (mNode1 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << mConductance << " to " << mNode1 << "," << mNode1 << std::endl;
		system.addCompToSystemMatrix(mNode1, mNode1, Complex(mConductance, 0));
	}
	if (mNode2 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << mConductance << " to " << mNode2 << "," << mNode2 << std::endl;
		system.addCompToSystemMatrix(mNode2, mNode2, Complex(mConductance, 0));
	}
	// Set off diagonal entries
	if (mNode1 >= 0 && mNode2 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << -mConductance << " to " << mNode1 << "," << mNode2 << std::endl;
		system.addCompToSystemMatrix(mNode1, mNode2, Complex(-mConductance, 0));
		mLog.Log(Logger::Level::DEBUG) << "Add " << -mConductance << " to " << mNode2 << "," << mNode1 << std::endl;
		system.addCompToSystemMatrix(mNode2, mNode1, Complex(-mConductance, 0));
	}
}

Complex Components::DP::Resistor::getCurrent(SystemModel& model) {
	Real realVolt = 0, imagVolt = 0;
	if (mNode1 >= 0)
	{
		realVolt += model.getCompFromLeftSideVector(mNode1).real();
		imagVolt += model.getCompFromLeftSideVector(mNode2).imag();
	}
	if (mNode2 >= 0)
	{
		realVolt -= model.getCompFromLeftSideVector(mNode1).real();
		imagVolt -= model.getCompFromLeftSideVector(mNode2).imag();
	}
	return Complex(realVolt*mConductance, imagVolt*mConductance);
}
