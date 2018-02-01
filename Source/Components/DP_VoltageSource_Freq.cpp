/** Real voltage source freq
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

#include "DP_VoltageSource_Freq.h"

using namespace DPsim;

Components::DP::VoltageSourceFreq::VoltageSourceFreq(String name, Int node1, Int node2, Complex voltage,
	Real resistance, Real omegaSource, Real switchTime, Real rampTime)
	: Component(name, node1, node2) {
	mResistance = resistance;
	mConductance = 1. / resistance;
	mVoltageAmp = std::abs(voltage);
	mVoltagePhase = std::arg(voltage);
	mSwitchTime = switchTime;
	mOmegaSource = omegaSource;
	mRampTime = rampTime;
	mVoltage = voltage;
	mCurrent = mVoltage / mResistance;
}

Components::DP::VoltageSourceFreq::VoltageSourceFreq(String name, Int node1, Int node2, Real voltageMag, Real voltagePhase,
	Real resistance, Real omegaSource, Real switchTime, Real rampTime)
	: VoltageSourceFreq(name, node1, node2, MathLibrary::polar(voltageMag, voltagePhase), resistance, omegaSource, switchTime, rampTime) {
}

void Components::DP::VoltageSourceFreq::applySystemMatrixStamp(SystemModel& system)
{
	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, Complex(mConductance, 0));
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, Complex(mConductance, 0));
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, Complex(-mConductance, 0));
		system.addCompToSystemMatrix(mNode2, mNode1, Complex(-mConductance, 0));
	}
}

void Components::DP::VoltageSourceFreq::applyRightSideVectorStamp(SystemModel& system)
{
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrent);
	}
}

void Components::DP::VoltageSourceFreq::step(SystemModel& system, Real time)
{
	if (time >= mSwitchTime && time < mSwitchTime + mRampTime) {
		Real fadeInOut = 0.5 + 0.5 * sin((time - mSwitchTime) / mRampTime * PI + -PI / 2);
		mVoltage = MathLibrary::polar(mVoltageAmp, mVoltagePhase + fadeInOut * mOmegaSource * time);
		mCurrent = mVoltage / mResistance;
	}
	else if (time >= mSwitchTime + mRampTime) {
		mVoltage = MathLibrary::polar(mVoltageAmp, mVoltagePhase + mOmegaSource * time);
		mCurrent = mVoltage / mResistance;
	}
	else {
		mVoltage = MathLibrary::polar(mVoltageAmp, mVoltagePhase);
		mCurrent = mVoltage / mResistance;
	}

	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrent);
	}

	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrent);
	}
}

Complex Components::DP::VoltageSourceFreq::getCurrent(SystemModel& system) {
	Complex Current;
	if (mNode1 >= 0) {
		Current += system.getCompFromLeftSideVector(mNode1) * mConductance;
	}

	if (mNode2 >= 0) {
		Current -= system.getCompFromLeftSideVector(mNode2) * mConductance;
	}
	return Current;
}
