#include "Inductor2.h"

Inductor2::Inductor2(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->inductance = inductance;
}

void Inductor2::applyMatrixStamp(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	double a = dt/(2*inductance);
	double b = dt*om/2;

	glr = a / (1 + b*b);
	gli = -a*b / (1 + b*b);
	pr = (1 - b*b) / (1 + b*b);
	pi = 2 * b / (1 + b*b);


	if (node1 >= 0) {
		g(node1, node1) = g(node1, node1) + glr;
		g(compOffset + node1, compOffset + node1) = g(compOffset + node1, compOffset + node1) + glr;
		g(compOffset + node1, node1) = g(compOffset + node1, node1) + gli;
		g(node1, compOffset + node1) = g(node1, compOffset + node1) - gli;
	}

	else if (node2 >= 0) {
		g(node2, node2) = g(node2, node2) + glr;
		g(compOffset + node2, compOffset + node2) = g(compOffset + node2, compOffset + node2) + glr;
		g(compOffset + node2, node2) = g(compOffset + node2, node2) + gli;
		g(node2, compOffset + node2) = g(node2, compOffset + node2) - gli;
	}

	if (node1 >= 0 && node2 >= 0) {
		g(node1, node2) = g(node1, node2) - glr;
		g(compOffset + node1, compOffset + node2) = g(compOffset + node1, compOffset + node2) - glr;
		g(compOffset + node1, node2) = g(compOffset + node1, node2) - gli;
		g(node1, compOffset + node2) = g(node1, compOffset + node2) + gli;

		g(node2, node1) = g(node2, node1) - glr;
		g(compOffset + node2, compOffset + node1) = g(compOffset + node2, compOffset + node1) - glr;
		g(compOffset + node2, node1) = g(compOffset + node2, node1) - gli;
		g(node2, compOffset + node1) = g(node2, compOffset + node1) + gli;
	}
}

void Inductor2::Init(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt) {
	applyMatrixStamp(g, j, compOffset, om, dt);

	// Initialize internal state
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}


void Inductor2::Step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	

	// Initialize internal state
	cureqr = pr*currr + pi*curri + glr*deltavr - gli*deltavi;
	cureqi = -pi*currr + pr*curri + gli*deltavr + glr*deltavi;

	//cout << "cureq = " << cureq << endl;

	if (node1 >= 0) {
		j(node1, 0) = j(node1, 0) - cureqr;
		j(compOffset + node1, 0) = j(compOffset + node1, 0) - cureqi;
	}

	if (node2 >= 0) {
		j(node2, 0) = j(node2, 0) + cureqr;
		j(compOffset + node2, 0) = j(compOffset + node2, 0) + cureqi;
	}
}


void Inductor2::PostStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (node1 >= 0) {
		vposr = vt(node1, 0);
		vposi = vt(compOffset + node1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (node2 >= 0) {
		vnegr = vt(node2, 0);
		vnegi = vt(compOffset + node2, 0);
	}
	else {
		vnegr = 0;
		vnegi = 0;
	}
	deltavr = vposr - vnegr;
	deltavi = vposi - vnegi;

	//cout << "deltav = " << deltav << endl;

	currr = glr*deltavr - gli*deltavi + cureqr;
	curri = gli*deltavr + glr*deltavi + cureqi;

	//cout << "curr = " << curr << endl;
}
