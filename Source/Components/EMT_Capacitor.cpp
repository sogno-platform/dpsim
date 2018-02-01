/** Capacitor
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

#include "EMT_Capacitor.h"

using namespace DPsim;

Components::EMT::Capacitor::Capacitor(String name, Int node1, Int node2, Real capacitance)
	: Component(name, node1, node2) {
	mCapacitance = capacitance;
	attrMap["capacitance"] = { Attribute::Real, &mCapacitance };
	mEquivCurrent = 0;
	mCurrent = 0;
	mVoltage = 0;
}

/// Initialize internal state
void Components::EMT::Capacitor::initialize(SystemModel& system) {
	mEquivCond = system.getOmega() * mCapacitance;
}

void Components::EMT::Capacitor::applySystemMatrixStamp(SystemModel& system) {
	//mGcr = 2.0 * mCapacitance / system.getTimeStep();
	//mGci = system.getOmega() * mCapacitance;

	if (mNode1 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode1, mEquivCond);
	}
	if (mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode2, mNode2, mEquivCond);
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode2, -mEquivCond);
		system.addRealToSystemMatrix(mNode2, mNode1, -mEquivCond);
	}
}

void Components::EMT::Capacitor::step(SystemModel& system, Real time) {
	// Initialize internal state
	//mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
	//mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;
	mEquivCurrent = mCurrent + mEquivCond * mVoltage;

	if (mNode1 >= 0)	{
		system.addRealToRightSideVector(mNode1, mEquivCurrent);
	}
	if (mNode2 >= 0)	{
		system.addRealToRightSideVector(mNode2, -mEquivCurrent);
	}
}

void Components::EMT::Capacitor::postStep(SystemModel& system) {
	Real voltageNode1, voltageNode2;
	// extract solution
	if (mNode1 >= 0) {
		voltageNode1 = system.getRealFromLeftSideVector(mNode1);
	}
	else {
		voltageNode1 = 0;
	}
	if (mNode2 >= 0) {
		voltageNode2 = system.getRealFromLeftSideVector(mNode2);
	}
	else {
		voltageNode2 = 0;
	}
	//mCurrr = mGcr * mDeltavr - mGci * mDeltavi - mCureqr;
	//mCurri = mGci * mDeltavr + mGcr * mDeltavi - mCureqi;
	mVoltage = voltageNode1 - voltageNode2;
	mCurrent = mEquivCond * mVoltage - mEquivCurrent;
}

Real Components::EMT::Capacitor::getCurrent(const SystemModel& system) {
	return mCurrent;
}
