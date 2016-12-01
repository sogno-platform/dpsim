#include "InterfacedInductor.h"

InterfacedInductor::InterfacedInductor(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->inductance = inductance;
}

void InterfacedInductor::Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	// Initialize internal state
	currentRe = 0;
	currentIm = 0;
	voltageRe = 0;
	voltageIm = 0;
}

void InterfacedInductor::applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {

}

void InterfacedInductor::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Calculate current for this step
	currentStepRe = currentRe + dt * (1. / inductance * voltageRe + om * currentIm);
	currentStepIm = currentIm + dt * (1. / inductance * voltageIm - om * currentRe);

	// Update current source accordingly
	if (node1 >= 0) {
		j(node1, 0) = j(node1, 0) - currentStepRe;
		j(compOffset + node1, 0) = j(compOffset + node1, 0) - currentStepIm;
	}

	if (node2 >= 0) {
		j(node2, 0) = j(node2, 0) + currentStepRe;
		j(compOffset + node2, 0) = j(compOffset + node2, 0) + currentStepIm;
	}	
	currentRe = currentStepRe;
	currentIm = currentStepIm;
}

void InterfacedInductor::PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (node1 >= 0) {
		vposr = vt(node1, 0);
		vposi = vt(compOffset + node1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (node2 >= 0) {
		vnegr = vt(node2, 0);
		vnegi = vt(compOffset + node2, 0);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	voltageRe = vposr - vnegr;
	voltageIm = vposi - vnegi;
}

