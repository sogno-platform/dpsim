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

#include "EMT_VoltageSource.h"

using namespace DPsim;

Component::EMT::VoltageSource::VoltageSource(String name, Int src, Int dest, Complex voltage, Real resistance)
	: VoltageSourceBase(name, src, dest, voltage)
{
	mResistance = resistance;
}

void Component::EMT::VoltageSource::applySystemMatrixStamp(SystemModel& system)
{
	mConductance = 1. / mResistance;
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

void Component::EMT::VoltageSource::applyRightSideVectorStamp(SystemModel& system)
{
	mCurrent = mVoltage.real() / mResistance;
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}

void Component::EMT::VoltageSource::step(SystemModel& system, Real time)
{
	mVoltageDiff = std::abs(mVoltage) * cos(std::arg(mVoltage) + system.getOmega() * time);
	mCurrent = mVoltageDiff / mResistance;

	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}
