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

void VoltSourceResEMT::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}


void VoltSourceResEMT::step(SystemModel& system, Real time) {
	mVoltageDiff = mVoltageAmp * cos(mVoltagePhase + system.getOmega() * time);
	mCurrent = mVoltageDiff / mResistance;

	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, mCurrent);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, -mCurrent);
	}
}
