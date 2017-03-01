#include "Capacitor.h"

Capacitor::Capacitor(std::string name, int src, int dest, double capacitance) : BaseComponent(name, src, dest) {
	this->capacitance = capacitance;
};	
		
void Capacitor::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
	double gcr = 2.0*capacitance/dt;
	double gci = om*capacitance;	

	if (mNode1 >= 0) {
		g(mNode1,mNode1) = g(mNode1,mNode1) + gcr;
		g(compOffset+mNode1,compOffset+mNode1) = g(compOffset+mNode1,compOffset+mNode1) + gcr;
		g(compOffset+mNode1,mNode1) = g(compOffset+mNode1,mNode1) + gci;
		g(mNode1,compOffset+mNode1) = g(mNode1,compOffset+mNode1) - gci;	
	}

	if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + gcr;
		g(compOffset + mNode2, compOffset + mNode2) = g(compOffset + mNode2, compOffset + mNode2) + gcr;
		g(compOffset + mNode2, mNode2) = g(compOffset + mNode2, mNode2) + gci;
		g(mNode2, compOffset + mNode2) = g(mNode2, compOffset + mNode2) - gci;
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		g(mNode1, mNode2) = g(mNode1, mNode2) - gcr;
		g(compOffset + mNode1, compOffset + mNode2) = g(compOffset + mNode1, compOffset + mNode2) - gcr;
		g(compOffset + mNode1, mNode2) = g(compOffset + mNode1, mNode2) - gci;
		g(mNode1, compOffset + mNode2) = g(mNode1, compOffset + mNode2) + gci;

		g(mNode2, mNode1) = g(mNode2, mNode1) - gcr;
		g(compOffset + mNode2, compOffset + mNode1) = g(compOffset + mNode2, compOffset + mNode1) - gcr;
		g(compOffset + mNode2, mNode1) = g(compOffset + mNode2, mNode1) - gci;
		g(mNode2, compOffset + mNode1) = g(mNode2, compOffset + mNode1) + gci;
	}
}

/// Initialize internal state
void Capacitor::init(double om, double dt) {
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}

void Capacitor::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Initialize internal state
	cureqr =  currr + 2.0*capacitance/dt*deltavr+om*capacitance*deltavi;
	cureqi =  curri + 2.0*capacitance/dt*deltavi-om*capacitance*deltavr;

	//cout << "cureq = " << cureq << endl;

	if (mNode1 >= 0)	{
		j(mNode1, 0) = j(mNode1, 0) + cureqr;
		j(compOffset+mNode1, 0) = j(compOffset+mNode1, 0) + cureqi;
	}

	if (mNode2 >= 0)	{
		j(mNode2, 0) = j(mNode2, 0) - cureqr;
		j(compOffset+mNode2, 0) = j(compOffset+mNode2, 0) - cureqi;
	}
}


void Capacitor::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;
	double gcr = 2.0*capacitance/dt;
	double gci = om*capacitance;			

	// extract solution
	if (mNode1 >= 0) {
		vposr = vt(mNode1, 0);
		vposi = vt(compOffset+mNode1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}
	if (mNode2 >= 0) {
		vnegr = vt(mNode2, 0);
		vnegi = vt(compOffset+mNode2, 0);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	deltavr = vposr-vnegr;
	deltavi = vposi-vnegi;

	//cout << "deltav = " << deltav << endl;

	currr =  gcr*deltavr-gci*deltavi-cureqr;
	curri =  gci*deltavr+gcr*deltavi-cureqi;

	//cout << "curr = " << curr << endl;
}
