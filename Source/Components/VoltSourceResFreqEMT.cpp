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

void VoltSourceResFreqEMT::applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + mCurrent;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - mCurrent;
	}
}


void VoltSourceResFreqEMT::step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t) {
	if (t >= mSwitchTime && t < mSwitchTime + mRampTime) {
		Real fadeInOut = 0.5 + 0.5 * sin( (t - mSwitchTime) / mRampTime * PI + - PI / 2);
		mVoltageDiff = mVoltageAmp*cos(mVoltagePhase + (om + fadeInOut * mOmegaSource) * t);
		mCurrent = mVoltageDiff / mResistance;
	}
	else if (t >= mSwitchTime + mRampTime) {
		mVoltageDiff = mVoltageAmp*cos(mVoltagePhase + (om + mOmegaSource) * t);
		mCurrent = mVoltageDiff / mResistance;
	}
	else {
		mVoltageDiff = mVoltageAmp*cos(mVoltagePhase + om*t);
		mCurrent = mVoltageDiff / mResistance;
	}
		
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + mCurrent;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - mCurrent;
	}
}
