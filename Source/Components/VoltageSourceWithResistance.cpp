#include "VoltageSourceWithResistance.h"

using namespace DPsim;

VoltageSourceWithResistance::VoltageSourceWithResistance(std::string name, int src, int dest, double voltage, double phase, double resistance) : BaseComponent(src, dest) {
	this->mName = name;
	this->voltageDiffr = voltage*cos(phase);
	this->voltageDiffi = voltage*sin(phase);
	this->resistance = resistance;		
	this->conductance = 1. / resistance;
	this->currentr = voltageDiffr / resistance;
	this->currenti = voltageDiffi / resistance;
}

void VoltageSourceWithResistance::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
	// Apply matrix stamp for equivalent resistance
	if (mNode1 >= 0) {
		g(mNode1, mNode1) = g(mNode1, mNode1) + conductance;
		g(compOffset + mNode1, compOffset + mNode1) = g(compOffset + mNode1, compOffset + mNode1) + conductance;
	}

	if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + conductance;
		g(compOffset + mNode2, compOffset + mNode2) = g(compOffset + mNode2, compOffset + mNode2) + conductance;
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		g(mNode1, mNode2) = g(mNode1, mNode2) - conductance;
		g(compOffset + mNode1, compOffset + mNode2) = g(compOffset + mNode1, compOffset + mNode2) - conductance;

		g(mNode2, mNode1) = g(mNode2, mNode1) - conductance;
		g(compOffset + mNode2, compOffset + mNode1) = g(compOffset + mNode2, compOffset + mNode1) - conductance;
	}
}

void VoltageSourceWithResistance::applyRightSideVectorStamp(DPSMatrix& j, int compOffset, double om, double dt) {
	// Apply matrix stamp for equivalent current source
	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + currentr;
		j(mNode1 + compOffset, 0) = j(compOffset + mNode1, 0) + currenti;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - currentr;
		j(mNode2 + compOffset, 0) = j(compOffset + mNode2, 0) - currenti;
	}
}


void VoltageSourceWithResistance::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {

	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) + currentr;
		j(mNode1 + compOffset, 0) = j(mNode1 + compOffset, 0) + currenti;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) - currentr;
		j(mNode2 + compOffset, 0) = j(mNode2 + compOffset, 0) - currenti;
	}
}
