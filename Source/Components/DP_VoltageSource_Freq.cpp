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

Components::DP::VoltageSourceFreq::VoltageSourceFreq(String name, Int src, Int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime, Real rampTime)
	: Base(name, src, dest)
{
	mResistance = resistance;
	mConductance = 1. / resistance;
	mVoltageAmp = voltage;
	mVoltagePhase = phase;
	mSwitchTime = switchTime;
	mOmegaSource = omegaSource;
	mRampTime = rampTime;
	mVoltageDiffr = mVoltageAmp*cos(mVoltagePhase);
	mVoltageDiffi = mVoltageAmp*sin(mVoltagePhase);
	mCurrentr = mVoltageDiffr / mResistance;
	mCurrenti = mVoltageDiffi / mResistance;
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
		system.addCompToRightSideVector(mNode1, Complex(mCurrentr, mCurrenti));
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, Complex(-mCurrentr, -mCurrenti));
	}
}

void Components::DP::VoltageSourceFreq::step(SystemModel& system, Real time)
{
	if (time >= mSwitchTime && time < mSwitchTime + mRampTime) {
		Real fadeInOut = 0.5 + 0.5 * sin((time - mSwitchTime) / mRampTime * PI + -PI / 2);
		mVoltageDiffr = mVoltageAmp*cos(mVoltagePhase + fadeInOut * mOmegaSource * time);
		mVoltageDiffi = mVoltageAmp*sin(mVoltagePhase + fadeInOut * mOmegaSource * time);
		mCurrentr = mVoltageDiffr / mResistance;
		mCurrenti = mVoltageDiffi / mResistance;
	}
	else if (time >= mSwitchTime + mRampTime) {
		mVoltageDiffr = mVoltageAmp*cos(mVoltagePhase + mOmegaSource * time);
		mVoltageDiffi = mVoltageAmp*sin(mVoltagePhase + mOmegaSource * time);
		mCurrentr = mVoltageDiffr / mResistance;
		mCurrenti = mVoltageDiffi / mResistance;
	}
	else {
		mVoltageDiffr = mVoltageAmp*cos(mVoltagePhase);
		mVoltageDiffi = mVoltageAmp*sin(mVoltagePhase);
		mCurrentr = mVoltageDiffr / mResistance;
		mCurrenti = mVoltageDiffi / mResistance;
	}

	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, Complex(mCurrentr, mCurrenti));
	}

	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, Complex(-mCurrentr, -mCurrenti));
	}
}

Complex Components::DP::VoltageSourceFreq::getCurrent(SystemModel& system)
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
