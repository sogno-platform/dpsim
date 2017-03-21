#include "InterfacedInductor.h"

using namespace DPsim;

InterfacedInductor::InterfacedInductor(std::string name, int src, int dest, Real inductance) : BaseComponent(name, src, dest) {
	this->mInductance = inductance;
}


/// Initialize internal state
void InterfacedInductor::init(Real om, Real dt) {
	mCurrentRe = 0;
	mCurrentIm = 0;
	mVoltageRe = 0;
	mVoltageIm = 0;
}


void InterfacedInductor::step(SystemModel& system, Real time) {
	// Calculate current for this step
	mCurrentStepRe = mCurrentRe + system.getTimeStep() * (1. / mInductance * mVoltageRe + system.getOmega() * mCurrentIm);
	mCurrentStepIm = mCurrentIm + system.getTimeStep() * (1. / mInductance * mVoltageIm - system.getOmega() * mCurrentRe);

	// Update current source accordingly
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, -mCurrentStepRe, -mCurrentStepIm);
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, mCurrentStepRe, mCurrentStepIm);
	}	
	mCurrentRe = mCurrentStepRe;
	mCurrentIm = mCurrentStepIm;
}

void InterfacedInductor::postStep(SystemModel& system) {
	double vposr, vnegr, vposi, vnegi;

	// extract solution
	if (mNode1 >= 0) {
		vposr = system.getRealFromLeftSideVector(mNode1);
		vposi = system.getImagFromLeftSideVector(mNode1);
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
	mVoltageRe = vposr - vnegr;
	mVoltageIm = vposi - vnegi;
}

