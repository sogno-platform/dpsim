#include "InterfacedInductor.h"

InterfacedInductor::InterfacedInductor(std::string name, int src, int dest, double inductance) : InterfaceCurrentSource(name, src, dest) {
	this->inductance = inductance;
}

void InterfacedInductor::Init(int compOffset, double om, double dt) {
	// Initialize internal state
	currentRe = 0;
	currentIm = 0;
	voltageRe = 0;
	voltageIm = 0;
}


void InterfacedInductor::Step(DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Calculate current for this step
	currentStepRe = currentRe + dt * (1. / inductance * voltageRe + om * currentIm);
	currentStepIm = currentIm + dt * (1. / inductance * voltageRe - om * currentRe);

	// Update current source accordingly
	if (node1 >= 0) {
		j(node1, 0) = j(node1, 0) - currentStepRe;
		j(compOffset + node1, 0) = j(compOffset + node1, 0) - currentStepIm;
	}

	if (node2 >= 0) {
		j(node2, 0) = j(node2, 0) + currentStepRe;
		j(compOffset + node2, 0) = j(compOffset + node2, 0) + currentStepIm;
	}
}

