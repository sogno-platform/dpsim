#include "InductorEMT.h"

using namespace DPsim;

InductorEMT::InductorEMT(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	mInductance = inductance;
	attrMap["inductance"] = {AttrReal, &mInductance};
}

void InductorEMT::applySystemMatrixStamp(SystemModel& system) {
	mGl = system.getTimeStep() / (2.0 * mInductance);

	if (mNode1 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode1, mGl);
	}
	if (mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode2, mNode2, mGl);		
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode2, -mGl);
		system.addRealToSystemMatrix(mNode2, mNode1, -mGl);		
	}
}


void InductorEMT::init(Real om, Real dt) {
	mCurr = 0;	
	mCureq = 0;	
	mDeltav = 0;	
}


void InductorEMT::step(SystemModel& system, Real time) {
	// Initialize internal state
	mCureq = mGl * mDeltav + mCurr;
		
	if (mNode1 >= 0) {
		system.addRealToRightSideVector(mNode1, -mCureq);
	}
	if (mNode2 >= 0) {
		system.addRealToRightSideVector(mNode2, mCureq);
	}
}


void InductorEMT::postStep(SystemModel& system) {
	Real vpos, vneg;	

	if (mNode1 >= 0) {
		vpos = system.getRealFromLeftSideVector(mNode1);	
	}
	else {
		vpos = 0;		
	}
	if (mNode2 >= 0) {
		vneg = system.getRealFromLeftSideVector(mNode2);
	}
	else {
		vneg = 0;		
	}
	mDeltav = vpos - vneg;	
	mCurr = mGl * mDeltav + mCureq;
}
