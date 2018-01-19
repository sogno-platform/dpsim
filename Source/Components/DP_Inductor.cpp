/**
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

Components::DP::Inductor::Inductor(String name, Int node1, Int node2, Real inductance, Logger::Level logLevel)
	: Base(name, node1, node2, logLevel) {
	mInductance = inductance;
	attrMap["inductance"] = {Attribute::Real, &mInductance};
	mEquivCurrent = { 0, 0 };
	mCurrent = { 0, 0 };
	mVoltage = { 0, 0 };
	mLog.Log(Logger::Level::DEBUG) << "Create Inductor " << name << " at " << mNode1 << "," << mNode2 << std::endl;
}

void Components::DP::Inductor::initialize(SystemModel& system) {
	Real a = system.getTimeStep() / (2. * mInductance);
	Real b = system.getTimeStep() * system.getOmega() / 2.;
	Complex impedance = { 0, system.getOmega() * mInductance };

	mEquivCond = { a / (1 + b*b), -a*b / (1 + b*b) };
	mPrevCurrFac = { (1 - b*b) / (1 + b*b), -2. * b / (1 + b*b) };
	mCurrent = mVoltage / impedance;
}

void Components::DP::Inductor::applySystemMatrixStamp(SystemModel& system) {
	//mGlr = a / (1 + b*b);
	//mGli = -a*b / (1 + b*b);
	//mPrevCurFacRe = (1 - b*b) / (1 + b*b);
	//mPrevCurFacIm = - 2. * b / (1 + b*b);

	if (mNode1 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << mEquivCond << " to " << mNode1 << "," << mNode1 << std::endl;
		system.addCompToSystemMatrix(mNode1, mNode1, mEquivCond);
	}
	if (mNode2 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << mEquivCond << " to " << mNode2 << "," << mNode2 << std::endl;
		system.addCompToSystemMatrix(mNode2, mNode2, mEquivCond);
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		mLog.Log(Logger::Level::DEBUG) << "Add " << -mEquivCond << " to " << mNode1 << "," << mNode2 << std::endl;
		system.addCompToSystemMatrix(mNode1, mNode2, -mEquivCond);
		mLog.Log(Logger::Level::DEBUG) << "Add " << -mEquivCond << " to " << mNode2 << "," << mNode1 << std::endl;
		system.addCompToSystemMatrix(mNode2, mNode1, -mEquivCond);
	}
}

void Components::DP::Inductor::step(SystemModel& system, Real time) {
	// Calculate equivalent current source for this time step
	//mCurEqRe = mGlr * mDeltaVre - mGli * mDeltaVim + mPrevCurFacRe * mCurrRe - mPrevCurFacIm * mCurrIm;
	//mCurEqIm = mGli * mDeltaVre + mGlr * mDeltaVim + mPrevCurFacIm * mCurrRe + mPrevCurFacRe * mCurrIm;
	mEquivCurrent = mEquivCond * mVoltage + mPrevCurrFac * mCurrent;

	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, -mEquivCurrent);
	}
	if (mNode2 >= 0)	{
		system.addCompToRightSideVector(mNode2, mEquivCurrent);
	}
}


void Components::DP::Inductor::postStep(SystemModel& system) {
	Complex voltNode1;
	Complex voltNode2;

	// extract solution
	if (mNode1 >= 0) {
		system.getRealFromLeftSideVector(mNode1);
		voltNode1 = system.getCompFromLeftSideVector(mNode1);
	}
	else {
		voltNode1 = { 0, 0};
	}

	if (mNode2 >= 0) {
		system.getRealFromLeftSideVector(mNode2);
		voltNode2 = system.getCompFromLeftSideVector(mNode2);
	}
	else {
		voltNode2 = { 0, 0 };
	}
	//mDeltaVre = vposr - vnegr;
	//mDeltaVim = vposi - vnegi;
	//mCurrRe = mGlr * mDeltaVre - mGli * mDeltaVim + mCurEqRe;
	//mCurrIm = mGli * mDeltaVre + mGlr * mDeltaVim + mCurEqIm;
	mVoltage = voltNode1 - voltNode2;
	mCurrent = mEquivCond * mVoltage + mEquivCurrent;
}

Complex Components::DP::Inductor::getCurrent(const SystemModel& system) {
	return mCurrent;
}
