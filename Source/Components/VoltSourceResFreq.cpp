#include "VoltSourceResFreq.h"

using namespace DPsim;

VoltSourceResFreq::VoltSourceResFreq(std::string name, int src, int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime, Real rampTime) : BaseComponent(src, dest) {
	mName = name;
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

void VoltSourceResFreq::applySystemMatrixStamp(SystemModel& system) {
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

void VoltSourceResFreq::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrentr, mCurrenti);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrentr, -mCurrenti);
	}
}


void VoltSourceResFreq::step(SystemModel& system, Real time) {
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
		system.addCompToRightSideVector(mNode1, mCurrentr, mCurrenti);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrentr, -mCurrenti);
	}
}
