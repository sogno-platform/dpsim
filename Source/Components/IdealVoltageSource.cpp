#include "IdealVoltageSource.h"

using namespace DPsim;

IdealVoltageSource::IdealVoltageSource(std::string name, int src, int dest, Real voltage, Real phase, int num) : BaseComponent(name, src, dest) {
	this->number = num;
	this->mVoltageDiffr = voltage*cos(phase);
	this->mVoltageDiffi = voltage*sin(phase);
}

void IdealVoltageSource::applySystemMatrixStamp(SystemModel& system) {
	
		
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
	Logger log;
	
	log.Log() << system.getCurrentSystemMatrix() << std::endl;
	log.WriteLogToFile("test.log");
}

void IdealVoltageSource::applyRightSideVectorStamp(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	system.addToRightSideVector(system.getCompOffset() - number, mVoltageDiffr);
	system.addToRightSideVector(2 * system.getCompOffset() - number, mVoltageDiffi);

	Logger log2;
	log2.Log() << j << std::endl;
	log2.WriteLogToFile("test2.log");
}

void IdealVoltageSource::step(SystemModel& system) {
	// Apply matrix stamp for equivalent current source
	system.addToRightSideVector(system.getCompOffset() - number, mVoltageDiffr);
	system.addToRightSideVector(2 * system.getCompOffset() - number, mVoltageDiffi);
}