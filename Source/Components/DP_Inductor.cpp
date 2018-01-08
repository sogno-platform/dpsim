/** InductorDP
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

#include "DP_Inductor.h"

using namespace DPsim;

Components::DP::Inductor::Inductor(String name, Int src, Int dest, Real inductance)
	: Base(name, src, dest)
{
	mInductance = inductance;
	attrMap["inductance"] = { Attribute::Real, &mInductance };
}

void Components::DP::Inductor::applySystemMatrixStamp(SystemModel& system)
{
	Real a = system.getTimeStep() / (2. * mInductance);
	Real b = system.getTimeStep() * system.getOmega() / 2.;
	mGlr = a / (1 + b*b);
	mGli = -a*b / (1 + b*b);
	mPrevCurFacRe = (1 - b*b) / (1 + b*b);
	mPrevCurFacIm = - 2. * b / (1 + b*b);

	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mGlr, mGli);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGlr, mGli);
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, -mGlr, -mGli);
		system.addCompToSystemMatrix(mNode2, mNode1, -mGlr, -mGli);
	}
}

void Components::DP::Inductor::init(Real om, Real dt)
{
	mCurrRe = 0;
	mCurrIm = 0;
	mCurEqRe = 0;
	mCurEqIm = 0;
	mDeltaVre = 0;
	mDeltaVim = 0;
}

void Components::DP::Inductor::step(SystemModel& system, Real time)
{
	// Initialize internal state
	mCurEqRe = mGlr * mDeltaVre - mGli * mDeltaVim + mPrevCurFacRe * mCurrRe - mPrevCurFacIm * mCurrIm;
	mCurEqIm = mGli * mDeltaVre + mGlr * mDeltaVim + mPrevCurFacIm * mCurrRe + mPrevCurFacRe * mCurrIm;

	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, -mCurEqRe, -mCurEqIm);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurEqRe, mCurEqIm);
	}
}

void Components::DP::Inductor::postStep(SystemModel& system)
{
	Real vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode1 >= 0) {
		system.getRealFromLeftSideVector(mNode1);
		vposr = system.getRealFromLeftSideVector(mNode1);
		vposi = system.getImagFromLeftSideVector(mNode1);
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (mNode2 >= 0) {
		system.getRealFromLeftSideVector(mNode2);
		vnegr = system.getRealFromLeftSideVector(mNode2);
		vnegi = system.getImagFromLeftSideVector(mNode2);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	mDeltaVre = vposr - vnegr;
	mDeltaVim = vposi - vnegi;
	mCurrRe = mGlr * mDeltaVre - mGli * mDeltaVim + mCurEqRe;
	mCurrIm = mGli * mDeltaVre + mGlr * mDeltaVim + mCurEqIm;
}

Complex Components::DP::Inductor::getCurrent(SystemModel& system)
{
	return Complex(mCurrRe, mCurrIm);
}
