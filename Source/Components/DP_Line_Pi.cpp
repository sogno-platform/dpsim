/** Pi Line
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

#include "DP_Line_Pi.h"

using namespace DPsim;

Component::DP::PiLine::PiLine(String name, Int node1, Int node2, Int node3, Real resistance, Real inductance, Real capacitance)
	: Base(name, node1, node2, node3)
{
	mResistance = resistance;
	mInductance = inductance;
	mCapacitance = capacitance;
	attrMap["resistance"]  = { Attribute::Real, &this->mResistance };
	attrMap["inductance"]  = { Attribute::Real, &this->mInductance };
	attrMap["capacitance"] = { Attribute::Real, &this->mCapacitance };
}

void Component::DP::PiLine::applySystemMatrixStamp(SystemModel& system)
{
	mConductance = 1.0 / mResistance;
	Real a = system.getTimeStep() / (2. * mInductance);
	Real b = system.getTimeStep() * system.getOmega() / 2.;
	mGlr = a / (1 + b*b);
	mGli = -a*b / (1 + b*b);
	mPrevCurFacRe = (1 - b*b) / (1 + b*b);
	mPrevCurFacIm = -2. * b / (1 + b*b);

	// Resistive part
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mConductance, 0);
	}
	if (mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode3, mConductance, 0);
	}
	if (mNode1 >= 0 && mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode3, -mConductance, 0);
		system.addCompToSystemMatrix(mNode3, mNode1, -mConductance, 0);
	}

	// Inductive part
	if (mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode3, mGlr, mGli);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGlr, mGli);
	}

	if (mNode3 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode2, -mGlr, -mGli);
		system.addCompToSystemMatrix(mNode2, mNode3, -mGlr, -mGli);
	}

	//capacitive part (only using half of nominal capaticance)
	mGcr = mCapacitance / system.getTimeStep();
	mGci = system.getOmega() * mCapacitance / 2;

	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mGcr, mGci);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGcr, mGci);
	}
}

void Component::DP::PiLine::init(Real om, Real dt)
{
	// Initialize internal state
	mCurrIndRe = 0;
	mCurrIndIm = 0;

	mCurrCapRe1 = 0;
	mCurrCapIm1 = 0;

	mCurrCapRe2 = 0;
	mCurrCapIm2 = 0;

	mCurEqIndRe = 0;
	mCurEqIndIm = 0;

	mCurEqCapRe1 = 0;
	mCurEqCapIm1 = 0;

	mCurEqCapRe2 = 0;
	mCurEqCapIm2 = 0;

	mDeltaVre = 0;
	mDeltaVim = 0;

	mVoltageAtNode1Re = 0;
	mVoltageAtNode1Im = 0;

	mVoltageAtNode2Re = 0;
	mVoltageAtNode2Im = 0;
}

void Component::DP::PiLine::step(SystemModel& system, Real time)
{
	// Calculate current source of inductor
	mCurEqIndRe = mGlr * mDeltaVre - mGli * mDeltaVim + mPrevCurFacRe * mCurrIndRe - mPrevCurFacIm * mCurrIndIm;
	mCurEqIndIm = mGli * mDeltaVre + mGlr * mDeltaVim + mPrevCurFacIm * mCurrIndRe + mPrevCurFacRe * mCurrIndIm;

	if (mNode3 >= 0) {
		system.addCompToRightSideVector(mNode3, -mCurEqIndRe, -mCurEqIndIm);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurEqIndRe, mCurEqIndIm);
	}

	// Calculate current source of capacitor 1
	mCurEqCapRe1 = mCurrCapRe1 + mGcr * mVoltageAtNode1Re + mGci * mVoltageAtNode1Im;
	mCurEqCapIm1 = mCurrCapIm1 + mGcr * mVoltageAtNode1Im - mGci * mVoltageAtNode1Re;

	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurEqCapRe1, mCurEqCapIm1);
	}

	// calculate curret source of capacitor 2
	mCurEqCapRe2 = mCurrCapRe2 + mGcr * mVoltageAtNode2Re + mGci * mVoltageAtNode2Im;
	mCurEqCapIm2 = mCurrCapIm2 + mGcr * mVoltageAtNode2Im - mGci * mVoltageAtNode2Re;
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurEqCapRe2, mCurEqCapIm2);
	}
}

void Component::DP::PiLine::postStep(SystemModel& system)
{
	Real vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode3 >= 0) {
		system.getRealFromLeftSideVector(mNode3);
		vposr = system.getRealFromLeftSideVector(mNode3);
		vposi = system.getImagFromLeftSideVector(mNode3);
	}

	if (mNode2 >= 0) {
		system.getRealFromLeftSideVector(mNode2);
		vnegr = system.getRealFromLeftSideVector(mNode2);
		vnegi = system.getImagFromLeftSideVector(mNode2);
	}

	mDeltaVre = vposr - vnegr;
	mDeltaVim = vposi - vnegi;
	mCurrIndRe = mGlr * mDeltaVre - mGli * mDeltaVim + mCurEqIndRe;
	mCurrIndIm = mGli * mDeltaVre + mGlr * mDeltaVim + mCurEqIndIm;

	// extract solution
	if (mNode1 >= 0) {
		mVoltageAtNode1Re = system.getRealFromLeftSideVector(mNode1);
		mVoltageAtNode1Im = system.getImagFromLeftSideVector(mNode1);
	}

	if (mNode2 >= 0) {
		mVoltageAtNode2Re = system.getRealFromLeftSideVector(mNode2);
		mVoltageAtNode2Im = system.getImagFromLeftSideVector(mNode2);
	}

	mCurrCapRe1 = mGcr * mVoltageAtNode1Re - mGci * mVoltageAtNode1Im - mCurEqCapRe1;
	mCurrCapIm1 = mGci * mVoltageAtNode1Re + mGcr * mVoltageAtNode1Im - mCurEqCapIm1;

	mCurrCapRe2 = mGcr * mVoltageAtNode2Re - mGci * mVoltageAtNode2Im - mCurEqCapRe2;
	mCurrCapIm2 = mGci * mVoltageAtNode2Re + mGcr * mVoltageAtNode2Im - mCurEqCapIm2;

}
