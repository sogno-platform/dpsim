#include "Inductor.h"

using namespace DPsim;

Inductor::Inductor(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	mInductance = inductance;
}	
		
void Inductor::applySystemMatrixStamp(SystemModel& system) {
	double a = system.getTimeStep() / (2. * mInductance);
	double b = system.getTimeStep() * system.getOmega() / 2.;
	mGlr = a / (1 + b*b);
	mGli = -a*b / (1 + b*b);
	mPrevCurFacRe = (1 - b*b) / (1 + b*b);
	mPrevCurFacIm = - 2. * b / (1 + b*b);
			 
	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mGlr, mGli);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGlr, mGli);
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, -mGlr, -mGli);
		system.addCompToSystemMatrix(mNode2, mNode1, -mGlr, -mGli);
	}
}


void Inductor::init(Real om, Real dt) {
	mCurrRe = 0;
	mCurrIm = 0;
	mCurEqRe = 0;
	mCurEqIm = 0;
	mDeltaVre = 0;
	mDeltaVim = 0;
}


void Inductor::step(SystemModel& system, Real time) {
	// Initialize internal state
	mCurEqRe = mGlr * mDeltaVre - mGli * mDeltaVim + mPrevCurFacRe * mCurrRe - mPrevCurFacIm * mCurrIm;
	mCurEqIm = mGli * mDeltaVre + mGlr * mDeltaVim + mPrevCurFacIm * mCurrRe + mPrevCurFacRe * mCurrIm;
		
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, -mCurEqRe, -mCurEqIm);
	}
	if (mNode2 >= 0)	{
		system.addCompToRightSideVector(mNode2, mCurEqRe, mCurEqIm);
	}
}


void Inductor::postStep(SystemModel& system) {
	double vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode1 >= 0)	{
		system.getRealFromLeftSideVector(mNode1);
		vposr = system.getRealFromLeftSideVector(mNode1);
		vposi = system.getImagFromLeftSideVector(mNode1);
	}
	else {
		vposr = 0;
		vposi = 0;
	}
	
	if (mNode2 >= 0) {
		system.getRealFromLeftSideVector(mNode2);
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
