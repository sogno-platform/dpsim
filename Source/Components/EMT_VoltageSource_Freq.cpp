/** Real voltage source freq (EMT)
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

#include "EMT_VoltageSource_Freq.h"

using namespace DPsim;

Component::EMT::VoltageSourceFreq::VoltageSourceFreq(String name, Int src, Int dest,
	Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime, Real rampTime)
	: Base(name, src, dest)
{
	mResistance = resistance;
	mConductance = 1. / resistance;
	mVoltageAmp = voltage;
	mVoltagePhase = phase;
	mSwitchTime = switchTime;
	mOmegaSource = omegaSource;
	mRampTime = rampTime;
	mVoltageDiff = mVoltageAmp*cos(mVoltagePhase);
	mCurrent = mVoltageDiff / mResistance;
}

void Component::EMT::VoltageSourceFreq::applySystemMatrixStamp(SystemModel& system)
{
	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode1, mConductance);
	}
	if (mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode2, mNode2, mConductance);
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode2, -mConductance);
		system.addRealToSystemMatrix(mNode2, mNode1, -mConductance);
	}
}

void Component::EMT::VoltageSourceFreq::applyRightSideVectorStamp(SystemModel& system)
{
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}

void Component::EMT::VoltageSourceFreq::step(SystemModel& system, Real time)
{
	if (time >= mSwitchTime && time < mSwitchTime + mRampTime) {
		Real fadeInOut = 0.5 + 0.5 * sin( (time - mSwitchTime) / mRampTime * PI + - PI / 2);
		mVoltageDiff = mVoltageAmp*cos(mVoltagePhase + (system.getOmega() + fadeInOut * mOmegaSource) * time);
		mCurrent = mVoltageDiff / mResistance;
	}
	else if (time >= mSwitchTime + mRampTime) {
		mVoltageDiff = mVoltageAmp*cos(mVoltagePhase + (system.getOmega() + mOmegaSource) * time);
		mCurrent = mVoltageDiff / mResistance;
	}
	else {
		mVoltageDiff = mVoltageAmp*cos(mVoltagePhase + system.getOmega() * time);
		mCurrent = mVoltageDiff / mResistance;
	}

	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}
