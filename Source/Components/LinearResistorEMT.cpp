#include "LinearResistorEMT.h"

using namespace DPsim;

LinearResistorEMT::LinearResistorEMT(std::string name, int src, int dest, Real resistance) : BaseComponent(name, src, dest) {
	this->mResistance = resistance;
	attrMap["resistance"] = {AttrReal, &this->mResistance};
}

void LinearResistorEMT::applySystemMatrixStamp(SystemModel& system) {
	this->mConductance = 1.0 / mResistance;
	// Set diagonal entries
	if (mNode1 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode1, mConductance);
	}
	if (mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode2, mNode2, mConductance);
	}
	// Set off diagonal entries
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addRealToSystemMatrix(mNode1, mNode2, -mConductance);
		system.addRealToSystemMatrix(mNode2, mNode1, -mConductance);
	}
}


