/** Real voltage source
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

#include "VoltSourceResDP.h"

using namespace DPsim;

VoltSourceRes::VoltSourceRes(String name, Int src, Int dest, Complex voltage, Real resistance) : BaseComponent(name, src, dest) {
	this->mVoltage = voltage;
	this->mResistance = resistance;
	attrMap["voltage"] = {AttrComplex, &this->mVoltage};
	attrMap["resistance"] = {AttrReal, &this->mResistance};
}

void VoltSourceRes::applySystemMatrixStamp(SystemModel& system) {
	mConductance = 1. / mResistance;
	mCurrentr = mVoltage.real() / mResistance;
	mCurrenti = mVoltage.imag() / mResistance;
	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mConductance, 0);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mConductance, 0);
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, -mConductance, 0);
		system.addCompToSystemMatrix(mNode2, mNode1, -mConductance, 0);
	}
}

void VoltSourceRes::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrentr, mCurrenti);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrentr, -mCurrenti);
	}
}

void VoltSourceRes::step(SystemModel& system, Real time) {
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrentr, mCurrenti);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrentr, -mCurrenti);
	}
}

Complex VoltSourceRes::getCurrent(SystemModel& system) {
	Real real = mCurrentr;
	Real imag = mCurrenti;
	if (mNode1 >= 0) {
		real += system.getRealFromLeftSideVector(mNode1)*mConductance;
		imag += system.getImagFromLeftSideVector(mNode1)*mConductance;
	}
	if (mNode2 >= 0) {
		real -= system.getRealFromLeftSideVector(mNode2)*mConductance;
		imag -= system.getImagFromLeftSideVector(mNode2)*mConductance;
	}
	return Complex(real, imag);
}
