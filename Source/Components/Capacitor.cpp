#include "Capacitor.h"

using namespace DPsim;

Capacitor::Capacitor(std::string name, Int src, Int dest, Real capacitance) : BaseComponent(name, src, dest) {
	this->capacitance = capacitance;
};	
		
void Capacitor::applySystemMatrixStamp(SystemModel& system) {
	mGcr = 2.0 * capacitance / system.getTimeStep();
	mGci = system.getOmega() * capacitance;	

	if (mNode1 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode1, mGcr, mGci);
	}
	if (mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode2, mNode2, mGcr, mGci);
	}
	if (mNode1 >= 0 && mNode2 >= 0) {
		system.addCompToSystemMatrix(mNode1, mNode2, -mGcr, -mGci);
		system.addCompToSystemMatrix(mNode2, mNode1, -mGcr, -mGci);
	}
}

/// Initialize internal state
void Capacitor::init(Real om, Real dt) {
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}

void Capacitor::step(SystemModel& system, Real time) {
	// Initialize internal state
	cureqr =  currr + mGcr * deltavr + mGci * deltavi;
	cureqi =  curri + mGcr * deltavi - mGci * deltavr;

	if (mNode1 >= 0)	{
		system.addCompToRightSideVector(mNode1, cureqr, cureqi);
	}
	if (mNode2 >= 0)	{
		system.addCompToRightSideVector(mNode2, -cureqr, -cureqi);
	}
}


void Capacitor::postStep(SystemModel& system) {
	double vposr, vnegr, vposi, vnegi;	

	// extract solution
	if (mNode1 >= 0) {
		vposr = system.getRealFromLeftSideVector(mNode1);
		vposi = system.getImagFromLeftSideVector(mNode1);
	}
	else {
		vposr = 0;
		vposi = 0;
	}
	if (mNode2 >= 0) {		
		vnegr = system.getRealFromLeftSideVector(mNode2);
		vnegi = system.getImagFromLeftSideVector(mNode2);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}

	deltavr = vposr - vnegr;
	deltavi = vposi - vnegi;
	currr = mGcr * deltavr - mGci * deltavi - cureqr;
	curri = mGci * deltavr + mGcr * deltavi - cureqi;
}
