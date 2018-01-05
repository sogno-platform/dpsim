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

#include "DP_VoltageSource.h"

using namespace DPsim;

Component::DP::VoltageSource::VoltageSource(String name, Int src, Int dest, Complex voltage, Real resistance)
	: Base(name, src, dest)
{
	mVoltage = voltage;
	mResistance = resistance;
	attrMap["voltage"]    = { Attribute::Complex, &mVoltage };
	attrMap["resistance"] = { Attribute::Real, &mResistance };
}

void Component::DP::VoltageSource::applySystemMatrixStamp(SystemModel& system)
{
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

void Component::DP::VoltageSource::applyRightSideVectorStamp(SystemModel& system)
{
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrentr, mCurrenti);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrentr, -mCurrenti);
	}
}

void Component::DP::VoltageSource::step(SystemModel& system, Real time)
{
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrentr, mCurrenti);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrentr, -mCurrenti);
	}
}

Complex Component::DP::VoltageSource::getCurrent(SystemModel& system)
{
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

void Component::DP::VoltageSource::setVoltage(Complex voltage)
{
	mVoltage = voltage;
}
