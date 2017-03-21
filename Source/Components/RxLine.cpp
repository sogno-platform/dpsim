#include "RxLine.h"

using namespace DPsim;

RxLine::RxLine(std::string name, int src, int dest, int newNode, Real resistance, Real inductance) : BaseComponent(name, src, dest, newNode) {
	this->mResistance = resistance;
	this->mConductance = 1.0 / resistance;
	this->mInductance = inductance;
}

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

void RxLine::step(SystemModel& system) {
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
