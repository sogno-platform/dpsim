#include "Capacitor.h"

Capacitor::Capacitor(std::string name, int src, int dest, double capacitance) : BaseComponent(src, dest) {
	this->capacitance = capacitance;
};	
		
void Capacitor::applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	double gcr = 2.0*capacitance/dt;
	double gci = om*capacitance;	

	if (node1 >= 0) {
		g(node1,node1) = g(node1,node1) + gcr;
		g(compOffset+node1,compOffset+node1) = g(compOffset+node1,compOffset+node1) + gcr;
		g(compOffset+node1,node1) = g(compOffset+node1,node1) + gci;
		g(node1,compOffset+node1) = g(node1,compOffset+node1) - gci;	
	}

	if (node2 >= 0) {
		g(node2, node2) = g(node2, node2) + gcr;
		g(compOffset + node2, compOffset + node2) = g(compOffset + node2, compOffset + node2) + gcr;
		g(compOffset + node2, node2) = g(compOffset + node2, node2) + gci;
		g(node2, compOffset + node2) = g(node2, compOffset + node2) - gci;
	}

	if (node1 >= 0 && node2 >= 0) {
		g(node1, node2) = g(node1, node2) - gcr;
		g(compOffset + node1, compOffset + node2) = g(compOffset + node1, compOffset + node2) - gcr;
		g(compOffset + node1, node2) = g(compOffset + node1, node2) - gci;
		g(node1, compOffset + node2) = g(node1, compOffset + node2) + gci;

		g(node2, node1) = g(node2, node1) - gcr;
		g(compOffset + node2, compOffset + node1) = g(compOffset + node2, compOffset + node1) - gcr;
		g(compOffset + node2, node1) = g(compOffset + node2, node1) - gci;
		g(node2, compOffset + node1) = g(node2, compOffset + node1) + gci;
	}
}
		
void Capacitor::Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	applyMatrixStamp(g, j, compOffset, om, dt);

	// Initialize internal state
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}

void Capacitor::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Initialize internal state
	cureqr =  currr + 2.0*capacitance/dt*deltavr+om*capacitance*deltavi;
	cureqi =  curri + 2.0*capacitance/dt*deltavi-om*capacitance*deltavr;

	//cout << "cureq = " << cureq << endl;

	if (node1 >= 0)	{
		j(node1, 0) = j(node1, 0) + cureqr;
		j(compOffset+node1, 0) = j(compOffset+node1, 0) + cureqi;
	}

	if (node2 >= 0)	{
		j(node2, 0) = j(node2, 1) - cureqr;
		j(compOffset+node2, 0) = j(compOffset+node2, 0) - cureqi;
	}
}


void Capacitor::PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;
	double gcr = 2.0*capacitance/dt;
	double gci = om*capacitance;			

	// extract solution
	if (node1 >= 0) {
		vposr = vt(node1, 0);
		vposi = vt(compOffset+node1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}
	if (node2 >= 0) {
		vnegr = vt(node2, 0);
		vnegi = vt(compOffset+node2, 0);
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
