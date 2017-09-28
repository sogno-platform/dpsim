#include "CurrentSource.h"

using namespace DPsim;

CurrentSource::CurrentSource(std::string name, int src, int dest, Complex current) : BaseComponent(name, src, dest) {
	this->mCurrent = current;
	attrMap["current"] = {AttrComplex, &this->mCurrent};
};
	
void CurrentSource::applyRightSideVectorStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		system.addCompToRightSideVector(mNode1, mCurrent.real(), mCurrent.imag());
	}
	if (mNode2 >= 0) {
		system.addCompToRightSideVector(mNode2, -mCurrent.real(), mCurrent.imag());
	}
}

void CurrentSource::step(SystemModel& system, Real time) {
	this->applyRightSideVectorStamp(system);
}

Complex CurrentSource::getCurrent(SystemModel &system) {
	return mCurrent;
}
