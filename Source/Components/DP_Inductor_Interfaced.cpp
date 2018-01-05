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

Component::DP::InterfacedInductor::InterfacedInductor(String name, Int src, Int dest, Real inductance)
	: Base(name, src, dest)
{
	this->mInductance = inductance;
	attrMap["inductance"] = { Attribute::Real, &mInductance };
}

/// Initialize internal state
void Component::DP::InterfacedInductor::init(Real om, Real dt)
{
	mCurrentRe = 0;
	mCurrentIm = 0;
	mVoltageRe = 0;
	mVoltageIm = 0;
}

void Component::DP::InterfacedInductor::step(SystemModel& system, Real time)
{
	// Calculate current for this step
	mCurrentStepRe = mCurrentRe + system.getTimeStep() * (1. / mInductance * mVoltageRe + system.getOmega() * mCurrentIm);
	mCurrentStepIm = mCurrentIm + system.getTimeStep() * (1. / mInductance * mVoltageIm - system.getOmega() * mCurrentRe);

	// Update current source accordingly
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, -mCurrentStepRe, -mCurrentStepIm);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurrentStepRe, mCurrentStepIm);
	}
	mCurrentRe = mCurrentStepRe;
	mCurrentIm = mCurrentStepIm;
}

void Component::DP::InterfacedInductor::postStep(SystemModel& system)
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
	mVoltageRe = vposr - vnegr;
	mVoltageIm = vposi - vnegi;
}

Complex Component::DP::InterfacedInductor::getCurrent(SystemModel& system)
{
	return Complex(mCurrentRe, mCurrentIm);
}
