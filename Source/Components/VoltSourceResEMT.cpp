#include "VoltSourceResEMT.h"

using namespace DPsim;

VoltSourceResEMT::VoltSourceResEMT(std::string name, int src, int dest, Real voltage, Real phase, Real resistance) : BaseComponent(name, src, dest) {
	mVoltageAmp = voltage;
	mVoltagePhase = phase;
	mResistance = resistance;
	mVoltageDiff = mVoltageAmp*cos(mVoltagePhase);
	mConductance = 1. / mResistance;
	mCurrent = mVoltageDiff / mResistance;
}

void VoltSourceResEMT::applySystemMatrixStamp(SystemModel& system) {
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

void VoltSourceResEMT::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + mCurrent;		
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - mCurrent;		
	}
}


void VoltSourceResEMT::step(SystemModel& system) {
	mVoltageDiff = mVoltageAmp*cos(mVoltagePhase + om * t);
	mCurrent = mVoltageDiff / mResistance;

	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + mCurrent;		
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - mCurrent;		
	}
}
