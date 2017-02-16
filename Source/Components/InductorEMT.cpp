#include "InductorEMT.h"

using namespace DPsim;

InductorEMT::InductorEMT(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->mInductance = inductance;
}

void InductorEMT::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
	double a = 2.0*mInductance / dt;
	double b = om*mInductance;

	mGl = a / (a*a + b*b);	
	mP = cos(2 * atan(om / (2 / dt)));
	

	if (mNode1 >= 0) {
		g(mNode1, mNode1) = g(mNode1, mNode1) + mGl;
	}

	if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + mGl;		
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		g(mNode1, mNode2) = g(mNode1, mNode2) - mGl;
		g(mNode2, mNode1) = g(mNode2, mNode1) - mGl;		
	}
}


/// Initialize internal state
void InductorEMT::init(double om, double dt) {
	mCurr = 0;	
	mCureq = 0;	
	mDeltav = 0;	
}


void InductorEMT::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Initialize internal state
	mCureq = mGl*mDeltav + mP*mCurr;
		
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) - mCureq;		
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) + mCureq;		
	}
}


void InductorEMT::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vpos, vneg;	

	if (mNode1 >= 0) {
		vpos = vt(mNode1, 0);		
	}
	else {
		vpos = 0;		
	}

	if (mNode2 >= 0) {
		vneg = vt(mNode2, 0);		
	}
	else {
		vneg = 0;		
	}
	mDeltav = vpos - vneg;	
	mCurr = mGl*mDeltav + mCureq;
}
