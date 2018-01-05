/** RX Line
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

#include "DP_Line_Rx.h"

using namespace DPsim;

Component::DP::RxLine::RxLine(String name, Int node1, Int node2, Real resistance, Real inductance, LineTypes type)
	: Base(name, node1, node2)
{
	mNumVirtualNodes = 1;
	mVirtualNodes = { 0 };
	mResistance = resistance;
	mConductance = 1.0 / resistance;
	mInductance = inductance;
	mType = type;
	attrMap["resistance"] = { Attribute::Real, &mResistance };
	attrMap["inductance"] = { Attribute::Real, &mInductance };
}

void Component::DP::RxLine::applySystemMatrixStamp(SystemModel& system)
{
	if (mType == LineTypes::RxLine2Node) {
		Real a = system.getTimeStep() / (2 * mInductance);
		Real b = system.getTimeStep()*system.getOmega() / 2;
		Real R = mResistance;

		mGlr = a*(1 + b*b + R*a + R*a*b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
		mGli = -a*b*(1 + b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);

		glr_ind = a / (1 + b*b);
		gli_ind = -a*b / (1 + b*b);
		mPrevCurFacRe = (1 - b*b) / (1 + b*b);
		mPrevCurFacIm = -2. * b / (1 + b*b);

		correctr = (1 + 2 * b*b + R*a + b*b*b*b + R*a*b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
		correcti = R*a*b*(1 + b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);


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
	else {

		Real a = system.getTimeStep() / (2. * mInductance);
		Real b = system.getTimeStep() * system.getOmega() / 2.;
		mGlr = a / (1 + b*b);
		mGli = -a*b / (1 + b*b);
		mPrevCurFacRe = (1 - b*b) / (1 + b*b);
		mPrevCurFacIm = -2. * b / (1 + b*b);

		// Resistive part
		// Set diagonal entries
		if (mNode1 >= 0) {
			system.addCompToSystemMatrix(mNode1, mNode1, mConductance, 0);
		}
		if (mVirtualNodes[0] >= 0) {
			system.addCompToSystemMatrix(mVirtualNodes[0], mVirtualNodes[0], mConductance, 0);
		}
		// Set off diagonal entries
		if (mNode1 >= 0 && mVirtualNodes[0] >= 0) {
			system.addCompToSystemMatrix(mNode1, mVirtualNodes[0], -mConductance, 0);
			system.addCompToSystemMatrix(mVirtualNodes[0], mNode1, -mConductance, 0);
		}

		// Inductance part
		// Set diagonal entries
		if (mVirtualNodes[0] >= 0) {
			system.addCompToSystemMatrix(mVirtualNodes[0], mVirtualNodes[0], mGlr, mGli);
		}
		if (mNode2 >= 0) {
			system.addCompToSystemMatrix(mNode2, mNode2, mGlr, mGli);
		}

		if (mVirtualNodes[0] >= 0 && mNode2 >= 0) {
			system.addCompToSystemMatrix(mVirtualNodes[0], mNode2, -mGlr, -mGli);
			system.addCompToSystemMatrix(mNode2, mVirtualNodes[0], -mGlr, -mGli);
		}
	}
}

void Component::DP::RxLine::init(Real om, Real dt)
{
	// Initialize internal state
	mCurrRe = 0;
	mCurrIm = 0;
	mCurEqRe = 0;
	mCurEqIm = 0;
	mDeltaVre = 0;
	mDeltaVim = 0;

	deltavr_ind = 0;
	deltavi_ind = 0;
	curri_ind = 0;
	currr_ind = 0;
	cureqr_ind = 0;
	cureqi_ind = 0;
}

void Component::DP::RxLine::step(SystemModel& system, Real time)
{
	if (mType == LineTypes::RxLine2Node) {

		// Initialize internal state
		cureqr_ind = mPrevCurFacRe*currr_ind - mPrevCurFacIm*mCurrIm + mGlr*deltavr_ind - mGli*deltavi_ind;
		cureqi_ind = mPrevCurFacIm*currr_ind + mPrevCurFacRe*mCurrIm + mGli*deltavr_ind + mGlr*deltavi_ind;

		mCurEqRe = cureqr_ind*correctr - cureqi_ind*correcti;
		mCurEqIm = cureqi_ind*correctr + correcti*cureqr_ind;

		//cout << "cureq = " << cureq << endl;

		if (mNode1 >= 0) {
			system.addCompToRightSideVector(mNode1, -mCurEqRe, -mCurEqIm);
		}

		if (mNode2 >= 0) {
			system.addCompToRightSideVector(mNode2, mCurEqRe, mCurEqIm);
		}

	}
	else {

		// Initialize internal state
		mCurEqRe = mGlr * mDeltaVre - mGli * mDeltaVim + mPrevCurFacRe * mCurrRe - mPrevCurFacIm * mCurrIm;
		mCurEqIm = mGli * mDeltaVre + mGlr * mDeltaVim + mPrevCurFacIm * mCurrRe + mPrevCurFacRe * mCurrIm;

		if (mVirtualNodes[0] >= 0) {
			system.addCompToRightSideVector(mVirtualNodes[0], -mCurEqRe, -mCurEqIm);
		}
		if (mNode2 >= 0) {
			system.addCompToRightSideVector(mNode2, mCurEqRe, mCurEqIm);
		}
	}
}

void Component::DP::RxLine::postStep(SystemModel& system)
{
	if (mType == LineTypes::RxLine2Node) {
		Real vposr, vnegr;
		Real vposi, vnegi;

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

		mDeltaVre = vposr - vnegr;
		mDeltaVim = vposi - vnegi;

		mCurrRe = mGlr * mDeltaVre - mGli * mDeltaVim + mCurEqRe;
		mCurrIm = mGli * mDeltaVre + mGlr * mDeltaVim + mCurEqIm;

		deltavr_ind = vposr - mResistance*mCurrRe - vnegr;
		deltavi_ind = vposi - mResistance*mCurrIm - vnegi;

		currr_ind = mCurrRe;
		curri_ind = mCurrIm;
	}
	else {
		Real vposr, vnegr, vposi, vnegi;

		// extract solution
		if (mVirtualNodes[0] >= 0) {
			system.getRealFromLeftSideVector(mVirtualNodes[0]);
			vposr = system.getRealFromLeftSideVector(mVirtualNodes[0]);
			vposi = system.getImagFromLeftSideVector(mVirtualNodes[0]);
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
}

Complex Component::DP::RxLine::getCurrent(SystemModel& system)
{
	return Complex(mCurrRe, mCurrIm);
}
