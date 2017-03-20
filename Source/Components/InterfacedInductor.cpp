#include "InterfacedInductor.h"

using namespace DPsim;

InterfacedInductor::InterfacedInductor(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->inductance = inductance;
}


/// Initialize internal state
void InterfacedInductor::init(int compOffset, double om, double dt) {
	currentRe = 0;
	currentIm = 0;
	voltageRe = 0;
	voltageIm = 0;
}


void InterfacedInductor::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Calculate current for this step
	currentStepRe = currentRe + dt * (1. / inductance * voltageRe + om * currentIm);
	currentStepIm = currentIm + dt * (1. / inductance * voltageIm - om * currentRe);

	// Update current source accordingly
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) - currentStepRe;
		j(compOffset + mNode1, 0) = j(compOffset + mNode1, 0) - currentStepIm;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) + currentStepRe;
		j(compOffset + mNode2, 0) = j(compOffset + mNode2, 0) + currentStepIm;
	}	
	currentRe = currentStepRe;
	currentIm = currentStepIm;
}

void InterfacedInductor::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (mNode1 >= 0) {
		vposr = vt(mNode1, 0);
		vposi = vt(compOffset + mNode1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (mNode2 >= 0) {
		vnegr = vt(mNode2, 0);
		vnegi = vt(compOffset + mNode2, 0);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	voltageRe = vposr - vnegr;
	voltageIm = vposi - vnegi;
}

