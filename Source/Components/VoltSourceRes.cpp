#include "VoltSourceRes.h"

using namespace DPsim;

VoltSourceRes::VoltSourceRes(std::string name, int src, int dest, Real voltage, Real phase, Real resistance) : BaseComponent(src, dest) {
	this->mName = name;
	this->mVoltageDiffr = voltage*cos(phase);
	this->mVoltageDiffi = voltage*sin(phase);
	this->mResistance = resistance;		
	this->mConductance = 1. / resistance;
	this->mCurrentr = mVoltageDiffr / resistance;
	this->mCurrenti = mVoltageDiffi / resistance;
}

void VoltSourceRes::applySystemMatrixStamp(DPSMatrix& g, int compOffset, Real om, Real dt) {
	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		g(mNode1, mNode1) = g(mNode1, mNode1) + mConductance;
		g(compOffset + mNode1, compOffset + mNode1) = g(compOffset + mNode1, compOffset + mNode1) + mConductance;
	}

	if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + mConductance;
		g(compOffset + mNode2, compOffset + mNode2) = g(compOffset + mNode2, compOffset + mNode2) + mConductance;
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		g(mNode1, mNode2) = g(mNode1, mNode2) - mConductance;
		g(compOffset + mNode1, compOffset + mNode2) = g(compOffset + mNode1, compOffset + mNode2) - mConductance;

		g(mNode2, mNode1) = g(mNode2, mNode1) - mConductance;
		g(compOffset + mNode2, compOffset + mNode1) = g(compOffset + mNode2, compOffset + mNode1) - mConductance;
	}
}

void VoltSourceRes::applyRightSideVectorStamp(DPSMatrix& j, int compOffset, Real om, Real dt) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + mCurrentr;
		j(mNode1 + compOffset, 0) = j(compOffset + mNode1, 0) + mCurrenti;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - mCurrentr;
		j(mNode2 + compOffset, 0) = j(compOffset + mNode2, 0) - mCurrenti;
	}
}


void VoltSourceRes::step(DPSMatrix& g, DPSMatrix& j, int compOffset, Real om, Real dt, Real t) {

	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + mCurrentr;
		j(mNode1 + compOffset, 0) = j(mNode1 + compOffset, 0) + mCurrenti;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - mCurrentr;
		j(mNode2 + compOffset, 0) = j(mNode2 + compOffset, 0) - mCurrenti;
	}
}
