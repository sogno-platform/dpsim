#include "CurrentSource.h"

using namespace DPsim;

CurrentSource::CurrentSource(std::string name, int src, int dest, double current, double phase) : BaseComponent(name, src, dest) {
	this->currentr = current*cos(phase);
	this->currenti = current*sin(phase);
};
	
void CurrentSource::applyRightSideVectorStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, currentr, currenti);		
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -currentr, -currenti);
	}
}

void CurrentSource::step(SystemModel& system, Real time) {
	this->applyRightSideVectorStamp(system);
}
