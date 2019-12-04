/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <cps/DP/DP_Ph1_VoltageSourceRamp.h>

using namespace CPS;

DP::Ph1::VoltageSourceRamp::VoltageSourceRamp(String uid, String name,
	Logger::Level logLevel)
	: PowerComponent<Complex>(uid, name, logLevel) {
	setVirtualNodeNumber(1);
	setTerminalNumber(2);
	mIntfVoltage = MatrixComp::Zero(1,1);
	mIntfCurrent = MatrixComp::Zero(1,1);

	addAttribute<Complex>("V_ref", &mVoltageRef, Flags::read | Flags::write);
	addAttribute<Real>("f_src", &mSrcFreq, Flags::read | Flags::write);
}

PowerComponent<Complex>::Ptr DP::Ph1::VoltageSourceRamp::clone(String name) {
	auto copy = VoltageSourceRamp::make(name, mLogLevel);
	copy->setParameters(mVoltageRef, mAddVoltage, mSrcFreq, mAddSrcFreq, mSwitchTime, mRampTime);
	return copy;
}

void DP::Ph1::VoltageSourceRamp::setParameters(Complex voltage, Complex addVoltage, Real srcFreq,
	Real addSrcFreq, Real switchTime, Real rampTime) {
	mVoltageRef = voltage;
	mAddVoltage = addVoltage;
	mSrcFreq = srcFreq;
	mAddSrcFreq = addSrcFreq;
	mSwitchTime = switchTime;
	mRampTime = rampTime;
}

void DP::Ph1::VoltageSourceRamp::initialize(Matrix frequencies) {
	PowerComponent<Complex>::initialize(frequencies);

	if (mVoltageRef == Complex(0, 0))
		mVoltageRef = initialSingleVoltage(1) - initialSingleVoltage(0);

	mSubVoltageSource = VoltageSource::make(mName + "_src", mLogLevel);
	mSubVoltageSource->setParameters(mVoltageRef, mSrcFreq);
	mSubVoltageSource->connect({ node(0), node(1) });
	mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[0], 0);
	mSubVoltageSource->initialize(frequencies);
}

void DP::Ph1::VoltageSourceRamp::initializeFromPowerflow(Real frequency) {
	mSubVoltageSource->initializeFromPowerflow(frequency);
}

void DP::Ph1::VoltageSourceRamp::mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
	MNAInterface::mnaInitialize(omega, timeStep);
	updateSimNodes();
	mSubVoltageSource->mnaInitialize(omega, timeStep, leftVector);
	// only need a new MnaPreStep that updates the reference voltage of mSubVoltageSource;
	// its own tasks then do the rest
	setAttributeRef("right_vector", mSubVoltageSource->attribute("right_vector"));
	mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
	for (auto task : mSubVoltageSource->mnaTasks()) {
		mMnaTasks.push_back(task);
	}
}

void DP::Ph1::VoltageSourceRamp::mnaApplySystemMatrixStamp(Matrix& systemMatrix) {
	mSubVoltageSource->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::VoltageSourceRamp::mnaApplyRightSideVectorStamp(Matrix& rightVector) {
	mSubVoltageSource->mnaApplyRightSideVectorStamp(rightVector);
}

void DP::Ph1::VoltageSourceRamp::updateState(Real time) {
	mIntfVoltage(0,0) = mVoltageRef;

	if (time >= mSwitchTime && time < mSwitchTime + mRampTime) {
		Real voltageAbs = Math::abs(mVoltageRef + (time - mSwitchTime) / mRampTime * mAddVoltage);
		Real voltagePhase = Math::phase(mVoltageRef + (time - mSwitchTime) / mRampTime * mAddVoltage);
		Real fadeInOut = 0.5 + 0.5 * sin((time - mSwitchTime) / mRampTime * PI + -PI / 2);
		mIntfVoltage(0,0) = Math::polar(voltageAbs, voltagePhase + fadeInOut * mAddSrcFreq * time);
	}
	else if (time >= mSwitchTime + mRampTime) {
		Real voltageAbs = Math::abs(mVoltageRef + mAddVoltage);
		Real voltagePhase = Math::phase(mVoltageRef + mAddVoltage);
		mIntfVoltage(0,0) = Math::polar(voltageAbs, voltagePhase + mAddSrcFreq * time);
	}
}

void DP::Ph1::VoltageSourceRamp::MnaPreStep::execute(Real time, Int timeStepCount) {
	mVoltageSource.updateState(time);
	mVoltageSource.mSubVoltageSource->attribute<Complex>("V_ref")->set(mVoltageSource.mIntfVoltage(0, 0));
}
