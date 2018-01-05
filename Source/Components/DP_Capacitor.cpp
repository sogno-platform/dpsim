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
	this->capacitance = capacitance;
	attrMap["capacitance"] = { Attribute::Real, &this->capacitance };
}

void Component::DP::Capacitor::applySystemMatrixStamp(SystemModel& system)
{
	mGcr = 2.0 * capacitance / system.getTimeStep();
	mGci = system.getOmega() * capacitance;

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
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}

void Component::DP::Capacitor::step(SystemModel& system, Real time)
{
	// Initialize internal state
	cureqr =  currr + mGcr * deltavr + mGci * deltavi;
	cureqi =  curri + mGcr * deltavi - mGci * deltavr;

	if (mNode1 >= 0)	{
		system.addCompToRightSideVector(mNode1, cureqr, cureqi);
	}
	if (mNode2 >= 0)	{
		system.addCompToRightSideVector(mNode2, -cureqr, -cureqi);
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

	deltavr = vposr - vnegr;
	deltavi = vposi - vnegi;
	currr = mGcr * deltavr - mGci * deltavi - cureqr;
	curri = mGci * deltavr + mGcr * deltavi - cureqi;
}

Complex Component::DP::Capacitor::getCurrent(SystemModel& system)
{
	return Complex(currr, curri);
}
