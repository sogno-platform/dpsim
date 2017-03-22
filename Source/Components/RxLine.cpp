#include "RxLine.h"

using namespace DPsim;

RxLine::RxLine(std::string name, int src, int dest, int node3, Real resistance, Real inductance) : BaseComponent(name, src, dest, node3) {
	this->mResistance = resistance;
	this->mConductance = 1.0 / resistance;
	this->mInductance = inductance;
}

//RxLine::RxLine(std::string name, int src, int dest, double resistance, double inductance) : BaseComponent(name, src, dest) {
//
//	this->resistance = resistance;
//	//this->conductance = 1.0 / resistance;
//	this->inductance = inductance;
//
//
//}
//
//
//void RxLine::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
//	double a = dt / (2 * inductance);
//	double b = dt*om / 2;
//	double R = resistance;
//
//	glr = a*(1 + b*b + R*a + R*a*b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//	gli = -a*b*(1 + b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//
//	glr_ind = a / (1 + b*b);
//	gli_ind = -a*b / (1 + b*b);
//	pr = (1 - b*b) / (1 + b*b);
//	pi = 2 * b / (1 + b*b);
//
//	correctr = (1 + 2 * b*b + R*a + b*b*b*b + R*a*b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//	correcti = R*a*b*(1 + b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//
//
//	if (mNode1 >= 0) {
//		g(mNode1, mNode1) = g(mNode1, mNode1) + glr;
//		g(compOffset + mNode1, compOffset + mNode1) = g(compOffset + mNode1, compOffset + mNode1) + glr;
//		g(compOffset + mNode1, mNode1) = g(compOffset + mNode1, mNode1) + gli;
//		g(mNode1, compOffset + mNode1) = g(mNode1, compOffset + mNode1) - gli;
//	}
//
//	if (mNode2 >= 0) {
//		g(mNode2, mNode2) = g(mNode2, mNode2) + glr;
//		g(compOffset + mNode2, compOffset + mNode2) = g(compOffset + mNode2, compOffset + mNode2) + glr;
//		g(compOffset + mNode2, mNode2) = g(compOffset + mNode2, mNode2) + gli;
//		g(mNode2, compOffset + mNode2) = g(mNode2, compOffset + mNode2) - gli;
//	}
//
//	if (mNode1 >= 0 && mNode2 >= 0) {
//		g(mNode1, mNode2) = g(mNode1, mNode2) - glr;
//		g(compOffset + mNode1, compOffset + mNode2) = g(compOffset + mNode1, compOffset + mNode2) - glr;
//		g(compOffset + mNode1, mNode2) = g(compOffset + mNode1, mNode2) - gli;
//		g(mNode1, compOffset + mNode2) = g(mNode1, compOffset + mNode2) + gli;
//
//		g(mNode2, mNode1) = g(mNode2, mNode1) - glr;
//		g(compOffset + mNode2, compOffset + mNode1) = g(compOffset + mNode2, compOffset + mNode1) - glr;
//		g(compOffset + mNode2, mNode1) = g(compOffset + mNode2, mNode1) - gli;
//		g(mNode2, compOffset + mNode1) = g(mNode2, compOffset + mNode1) + gli;
//	}
//
//
//}
//
//void RxLine::init(double om, double dt) {
//
//	// Initialize internal state
//	currr = 0;
//	curri = 0;
//	cureqr = 0;
//	cureqi = 0;
//	deltavr = 0;
//	deltavi = 0;
//
//	deltavr_ind = 0;
//	deltavi_ind = 0;
//	currr_ind = 0;
//	curri_ind = 0;
//	cureqr_ind = 0;
//	cureqi_ind = 0; 
//}
//
//
//void RxLine::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
//
//
//	// Initialize internal state
//	cureqr_ind = pr*currr_ind + pi*curri_ind + glr*deltavr_ind - gli*deltavi_ind;
//	cureqi_ind = -pi*currr_ind + pr*curri_ind + gli*deltavr_ind + glr*deltavi_ind;
//	
//	cureqr = cureqr_ind*correctr - cureqi_ind*correcti;
//	cureqi = cureqi_ind*correctr + correcti*cureqr_ind;
//
//	//cout << "cureq = " << cureq << endl;
//
//	if (mNode1 >= 0) {
//		j(mNode1, 0) = j(mNode1, 0) - cureqr;
//		j(compOffset + mNode1, 0) = j(compOffset + mNode1, 0) - cureqi;
//	}
//
//	if (mNode2 >= 0) {
//		j(mNode2, 0) = j(mNode2, 0) + cureqr;
//		j(compOffset + mNode2, 0) = j(compOffset + mNode2, 0) + cureqi;
//	}
//}
//
//void RxLine::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
//	double vposr, vnegr;
//	double vposi, vnegi;
//
//	// extract solution
//	if (mNode1 >= 0) {
//		vposr = vt(mNode1, 0);
//		vposi = vt(compOffset + mNode1, 0);
//	}
//	else {
//		vposr = 0;
//		vposi = 0;
//	}
//
//	if (mNode2 >= 0) {
//		vnegr = vt(mNode2, 0);
//		vnegi = vt(compOffset + mNode2, 0);
//	}
//	else {
//		vnegr = 0;
//		vnegi = 0;
//	}
//
//	deltavr = vposr - vnegr;
//	deltavi = vposi - vnegi;
//
//	currr = glr*deltavr - gli*deltavi + cureqr;
//	curri = gli*deltavr + glr*deltavi + cureqi;
//
//	deltavr_ind = vposr - resistance*currr - vnegr;
//	deltavi_ind = vposi - resistance*curri - vnegi;
//
//	currr_ind = glr_ind*deltavr_ind - gli_ind*deltavi_ind + cureqr_ind;
//	curri_ind = gli_ind*deltavr_ind + glr_ind*deltavi_ind + cureqi_ind;
//
//}

void RxLine::applySystemMatrixStamp(SystemModel& system) {
	Real a = system.getTimeStep() / (2. * mInductance);
	Real b = system.getTimeStep() * system.getOmega() / 2.;
	mGlr = a / (1 + b*b);
	mGli = -a*b / (1 + b*b);
	mPrevCurFacRe = (1 - b*b) / (1 + b*b);
	mPrevCurFacIm = 2 * b / (1 + b*b);

	// Resistive part
	// Set diagonal entries
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mConductance, 0);
	}
	if (mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode3, mConductance, 0);
	}
	// Set off diagonal entries
	if (mNode1 >= 0 && mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode3, -mConductance, 0);
		system.addCompToSystemMatrix(mNode3, mNode1, -mConductance, 0);
	}

	// Inductance part
	// Set diagonal entries
	if (mNode3 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode3, mGlr, mGli);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGlr, mGli);
	}
	// Set off diagonal entries
	if (mNode3 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode3, mNode2, -mGlr, -mGli);
		system.addCompToSystemMatrix(mNode2, mNode3, -mGlr, -mGli);
	}
}

void RxLine::init(Real om, Real dt) {
	// Initialize internal state
	mCurrRe = 0;
	mCurrIm = 0;
	mCurEqRe = 0;
	mCurEqIm = 0;
	mDeltaVre = 0;
	mDeltaVim = 0;
}

void RxLine::step(SystemModel& system, Real time) {
	// Initialize internal state
	mCurEqRe = mPrevCurFacRe * mCurrRe + mPrevCurFacIm * mCurrIm + mGlr * mDeltaVre - mGli * mDeltaVim;
	mCurEqIm = -mPrevCurFacIm * mCurrRe + mPrevCurFacRe * mCurrIm + mGli * mDeltaVre + mGlr * mDeltaVim;

	if (mNode3 >= 0) {
		system.addCompToRightSideVector(mNode3, -mCurEqRe, -mCurEqIm);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurEqRe, mCurEqIm);
	}
}

void RxLine::postStep(SystemModel& system) {
	Real vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode3 >= 0) {
		vposr = system.getRealFromLeftSideVector(mNode3);
		vposi = system.getImagFromLeftSideVector(mNode3);
	}
	else {
		vposr = 0;
		vposi = 0;
	}
	if (mNode2 >= 0) {
		vnegr = system.getRealFromLeftSideVector(mNode2);
		vnegi = system.getImagFromLeftSideVector(mNode2);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	mDeltaVre = vposr - vnegr;
	mDeltaVim = vposi - vnegi;
	mCurrRe = mGlr * mDeltaVre - mGli * mDeltaVim + mCurEqRe;
	mCurrIm = mGli * mDeltaVre + mGlr * mDeltaVim + mCurEqIm;
}

