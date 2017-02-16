#include "InductorEMT.h"

using namespace DPsim;

InductorEMT::InductorEMT(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->mInductance = inductance;
}

void InductorEMT::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
	double a = 2.0*mInductance / dt;
	double b = om*mInductance;

	mGlr = a / (a*a + b*b);
	mGli = -b / (a*a + b*b);
	mPr = cos(2 * atan(om / (2 / dt)));
	mPi = -sin(2 * atan(om / (2 / dt)));

	if (mNode1 >= 0) {
		g(mNode1, mNode1) = g(mNode1, mNode1) + mGlr;
	}

	if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + mGlr;		
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		g(mNode1, mNode2) = g(mNode1, mNode2) - mGlr;
		g(mNode2, mNode1) = g(mNode2, mNode1) - mGlr;		
	}
}


/// Initialize internal state
void InductorEMT::init(double om, double dt) {
	mCurrr = 0;
	mCurri = 0;
	mCureqr = 0;
	mCureqi = 0;
	mDeltavr = 0;
	mDeltavi = 0;
}


void InductorEMT::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Initialize internal state
	mCureqr = mGlr*mDeltavr - mGli*mDeltavi + mPr*mCurrr - mPi*mCurri;
	mCureqi = mGli*mDeltavr + mGlr*mDeltavi + mPi*mCurrr + mPr*mCurri;
	
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) - mCureqr;		
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) + mCureqr;		
	}
}


void InductorEMT::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	if (mNode1 >= 0) {
		vposr = vt(mNode1, 0);		
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (mNode2 >= 0) {
		vnegr = vt(mNode2, 0);		
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	mDeltavr = vposr - vnegr;	
	mCurrr = mGlr*mDeltavr + mCureqr;
}
