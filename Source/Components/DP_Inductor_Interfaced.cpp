/** Interfaced inductor
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

#include "DP_Inductor_Interfaced.h"

using namespace DPsim;

Components::DP::InterfacedInductor::InterfacedInductor(String name, Int node1, Int node2, Real inductance)
	: Base(name, node1, node2)
{
	mInductance = inductance;
	attrMap["inductance"] = { Attribute::Real, &mInductance };
}

/// Initialize internal state
void Components::DP::InterfacedInductor::initialize(SystemModel& system) {
	mCurrentRe = 0;
	mCurrentIm = 0;
	mVoltageRe = 0;
	mVoltageIm = 0;
}

void Components::DP::InterfacedInductor::step(SystemModel& system, Real time)
{
	// Calculate current for this step
	mCurrentStepRe = mCurrentRe + system.getTimeStep() * (1. / mInductance * mVoltageRe + system.getOmega() * mCurrentIm);
	mCurrentStepIm = mCurrentIm + system.getTimeStep() * (1. / mInductance * mVoltageIm - system.getOmega() * mCurrentRe);

	// Update current source accordingly
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, Complex(-mCurrentStepRe, -mCurrentStepIm));
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, Complex(mCurrentStepRe, mCurrentStepIm));
	}
	mCurrentRe = mCurrentStepRe;
	mCurrentIm = mCurrentStepIm;
}

void Components::DP::InterfacedInductor::postStep(SystemModel& system)
{
	Real vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode1 >= 0) {
		vposr = system.getCompFromLeftSideVector(mNode1).real();
		vposi = system.getCompFromLeftSideVector(mNode1).imag();
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (mNode2 >= 0) {
		vnegr = system.getCompFromLeftSideVector(mNode2).real();
		vnegi = system.getCompFromLeftSideVector(mNode2).imag();
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	mVoltageRe = vposr - vnegr;
	mVoltageIm = vposi - vnegi;
}

Complex Components::DP::InterfacedInductor::getCurrent(SystemModel& system)
{
	return Complex(mCurrentRe, mCurrentIm);
}
