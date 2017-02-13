#include "VoltageSourceWithResistance.h"

VoltageSourceWithResistance::VoltageSourceWithResistance(std::string name, int src, int dest, double voltage, double phase, double resistance) : BaseComponent(src, dest) {
	this->name = name;
	this->voltageDiffr = voltage*cos(phase);
	this->voltageDiffi = voltage*sin(phase);
	this->resistance = resistance;		
	this->conductance = 1. / resistance;
	this->currentr = voltageDiffr / resistance;
	this->currenti = voltageDiffi / resistance;
}

void VoltageSourceWithResistance::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
	// Apply matrix stamp for equivalent resistance
	if (node1 >= 0) {
		g(node1, node1) = g(node1, node1) + conductance;
		g(compOffset + node1, compOffset + node1) = g(compOffset + node1, compOffset + node1) + conductance;
	}

	if (node2 >= 0) {
		g(node2, node2) = g(node2, node2) + conductance;
		g(compOffset + node2, compOffset + node2) = g(compOffset + node2, compOffset + node2) + conductance;
	}

	if (node1 >= 0 && node2 >= 0) {
		g(node1, node2) = g(node1, node2) - conductance;
		g(compOffset + node1, compOffset + node2) = g(compOffset + node1, compOffset + node2) - conductance;

		g(node2, node1) = g(node2, node1) - conductance;
		g(compOffset + node2, compOffset + node1) = g(compOffset + node2, compOffset + node1) - conductance;
	}
}

void VoltageSourceWithResistance::applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) {
	// Apply matrix stamp for equivalent current source
	if (node1 >= 0) {
		j(node1, 0) = j(node1, 0) + currentr;
		j(node1, 0) = j(compOffset + node1, 0) + currenti;
	}

	if (node2 >= 0) {
		j(node2, 0) = j(node2, 0) - currentr;
		j(node2, 0) = j(compOffset+node2, 0) - currenti;
	}
}


void VoltageSourceWithResistance::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {

	if (node1 >= 0) {
		j(node1, 0) = j(node1, 0) + currentr;
		j( node1, 0) = j(node1, 0) + currenti;
	}

	if (node2 >= 0) {
		j(node2, 0) = j(node2, 0) - currentr;
		j(node2, 0) = j(compOffset + node2, 0) - currenti;
	}
}
