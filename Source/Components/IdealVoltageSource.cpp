#include "IdealVoltageSource.h"

using namespace DPsim;

IdealVoltageSource::IdealVoltageSource(std::string name, int src, int dest, Real voltage, Real phase, int num) : BaseComponent(name, src, dest) {
	this->number = num;
	this->mVoltageDiffr = voltage*cos(phase);
	this->mVoltageDiffi = voltage*sin(phase);
}

void IdealVoltageSource::applySystemMatrixStamp(SystemModel& system) {
	number = system.getNumIdealVS() - number + 1;
	if (mNode1 >= 0) {
		system.setSystemMatrixElement(system.getCompOffset() - number, mNode1, 1);
		system.setSystemMatrixElement(mNode1, system.getCompOffset() - number, 1);
		system.setSystemMatrixElement(2* system.getCompOffset() - number, mNode1 + system.getCompOffset(), 1);
		system.setSystemMatrixElement(mNode1 + system.getCompOffset(), 2 * system.getCompOffset() - number, 1);
	}

	if (mNode2 >= 0) {
		system.setSystemMatrixElement(system.getCompOffset() - number, mNode2, -1);
		system.setSystemMatrixElement(mNode2, system.getCompOffset() - number, -1);
		system.setSystemMatrixElement(2 * system.getCompOffset() - number, mNode2 + system.getCompOffset(), -1);
		system.setSystemMatrixElement(mNode2 + system.getCompOffset(), 2 * system.getCompOffset() - number, -1);
	}
}

void IdealVoltageSource::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	system.addRealToRightSideVector(system.getCompOffset() - number, mVoltageDiffr);
	system.addRealToRightSideVector(2 * system.getCompOffset() - number, mVoltageDiffi);
}

void IdealVoltageSource::step(SystemModel& system, Real time) {
	// Apply matrix stamp for equivalent current source
	system.addRealToRightSideVector(system.getCompOffset() - number, mVoltageDiffr);
	system.addRealToRightSideVector(2 * system.getCompOffset() - number, mVoltageDiffi);
}