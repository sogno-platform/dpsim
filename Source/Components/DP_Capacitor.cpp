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

#include "DP_Capacitor.h"

using namespace DPsim;

Component::DP::Capacitor::Capacitor(String name, Int src, Int dest, Real capacitance)
	: Base(name, src, dest)
{
	mCapacitance = capacitance;

	attrMap["capacitance"] = { Attribute::Real, &mCapacitance };
}

void Component::DP::Capacitor::applySystemMatrixStamp(SystemModel& system)
{
	mGcr = 2.0 * mCapacitance / system.getTimeStep();
	mGci = system.getOmega() * mCapacitance;

	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mGcr, mGci);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGcr, mGci);
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, -mGcr, -mGci);
		system.addCompToSystemMatrix(mNode2, mNode1, -mGcr, -mGci);
	}
}

/// Initialize internal state
void Component::DP::Capacitor::init(Real om, Real dt)
{
	mCurrr = 0;
	mCurri = 0;
	mCureqr = 0;
	mCureqi = 0;
	mDeltavr = 0;
	mDeltavi = 0;
}

void Component::DP::Capacitor::step(SystemModel& system, Real time)
{
	// Initialize internal state
	mCureqr = mCurrr + mGcr * mDeltavr + mGci * mDeltavi;
	mCureqi = mCurri + mGcr * mDeltavi - mGci * mDeltavr;

	if (mNode1 >= 0)	{
		system.addCompToRightSideVector(mNode1, mCureqr, mCureqi);
	}
	if (mNode2 >= 0)	{
		system.addCompToRightSideVector(mNode2, -mCureqr, -mCureqi);
	}
}

void Component::DP::Capacitor::postStep(SystemModel& system)
{
	Real vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode1 >= 0) {
		vposr = system.getRealFromLeftSideVector(mNode1);
		vposi = system.getImagFromLeftSideVector(mNode1);
	}
	else {
		vposr = 0;
		vposi = 0;
	}
	if (mNode2 >= 0) {
		vnegr = system.getRealFromLeftSideVector(mNode2);
		vnegi = system.getImagFromLeftSideVector(mNode2);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}

	mDeltavr = vposr - vnegr;
	mDeltavi = vposi - vnegi;

	mCurrr = mGcr * mDeltavr - mGci * mDeltavi - mCureqr;
	mCurri = mGci * mDeltavr + mGcr * mDeltavi - mCureqi;
}

Complex Component::DP::Capacitor::getCurrent(SystemModel& system)
{
	return Complex(mCurrr, mCurri);
}
