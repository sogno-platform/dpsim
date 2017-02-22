#include "Inductor2.h"

Inductor2::Inductor2(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->inductance = inductance;
}

void Inductor2::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
	double a = dt/(2*inductance);
	double b = dt*om/2;

	glr = a / (1 + b*b);
	gli = -a*b / (1 + b*b);
	pr = (1 - b*b) / (1 + b*b);
	pi = 2 * b / (1 + b*b);


	if (mNode1 >= 0) {
		g(mNode1, mNode1) = g(mNode1, mNode1) + glr;
		g(compOffset + mNode1, compOffset + mNode1) = g(compOffset + mNode1, compOffset + mNode1) + glr;
		g(compOffset + mNode1, mNode1) = g(compOffset + mNode1, mNode1) + gli;
		g(mNode1, compOffset + mNode1) = g(mNode1, compOffset + mNode1) - gli;
	}

	else if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + glr;
		g(compOffset + mNode2, compOffset + mNode2) = g(compOffset + mNode2, compOffset + mNode2) + glr;
		g(compOffset + mNode2, mNode2) = g(compOffset + mNode2, mNode2) + gli;
		g(mNode2, compOffset + mNode2) = g(mNode2, compOffset + mNode2) - gli;
	}

	if (mNode1 >= 0 && mNode2 >= 0) {
		g(mNode1, mNode2) = g(mNode1, mNode2) - glr;
		g(compOffset + mNode1, compOffset + mNode2) = g(compOffset + mNode1, compOffset + mNode2) - glr;
		g(compOffset + mNode1, mNode2) = g(compOffset + mNode1, mNode2) - gli;
		g(mNode1, compOffset + mNode2) = g(mNode1, compOffset + mNode2) + gli;

		g(mNode2, mNode1) = g(mNode2, mNode1) - glr;
		g(compOffset + mNode2, compOffset + mNode1) = g(compOffset + mNode2, compOffset + mNode1) - glr;
		g(compOffset + mNode2, mNode1) = g(compOffset + mNode2, mNode1) - gli;
		g(mNode2, compOffset + mNode1) = g(mNode2, compOffset + mNode1) + gli;
	}
}

void Inductor2::init(double om, double dt) {
	
	// Initialize internal state
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}


void Inductor2::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	

	// Initialize internal state
	cureqr = pr*currr + pi*curri + glr*deltavr - gli*deltavi;
	cureqi = -pi*currr + pr*curri + gli*deltavr + glr*deltavi;

	//cout << "cureq = " << cureq << endl;

	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) - cureqr;
		j(compOffset + mNode1, 0) = j(compOffset + mNode1, 0) - cureqi;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) + cureqr;
		j(compOffset + mNode2, 0) = j(compOffset + mNode2, 0) + cureqi;
	}
}


void Inductor2::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (mNode1 >= 0) {
		vposr = vt(mNode1, 0);
		vposi = vt(compOffset + mNode1, 0);
	}
	else {
		vposr = 0;
		vposi = 0;
	}

	if (mNode2 >= 0) {
		vnegr = vt(mNode2, 0);
		vnegi = vt(compOffset + mNode2, 0);
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
