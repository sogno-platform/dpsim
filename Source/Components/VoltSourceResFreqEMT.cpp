#include "VoltSourceResFreqEMT.h"

using namespace DPsim;

VoltSourceResFreqEMT::VoltSourceResFreqEMT(std::string name, int src, int dest, Real voltage, Real phase, Real resistance, Real omegaSource, Real switchTime, Real rampTime) : BaseComponent(src, dest) {
	mName = name;
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

void VoltSourceResFreqEMT::applySystemMatrixStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		g(mNode1, mNode1) = g(mNode1, mNode1) + mConductance;
	}

	if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + mConductance;
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		g(mNode1, mNode2) = g(mNode1, mNode2) - mConductance;
		g(mNode2, mNode1) = g(mNode2, mNode1) - mConductance;
	}
}

void VoltSourceResFreqEMT::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + mCurrent;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - mCurrent;
	}
}


void VoltSourceResFreqEMT::step(SystemModel& system, Real time) {
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
