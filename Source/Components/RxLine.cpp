#include "RxLine.h"

using namespace DPsim;


//RxLine::RxLine(std::string name, int src, int dest, double resistance, double inductance) : BaseComponent(name, src, dest) {
//
//	this->resistance = resistance;
//	//this->conductance = 1.0 / resistance;
//	this->inductance = inductance;
//
//
//}
//
//
//void RxLine::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
//	double a = dt / (2 * inductance);
//	double b = dt*om / 2;
//	double R = resistance;
//
//	glr = a*(1 + b*b + R*a + R*a*b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//	gli = -a*b*(1 + b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//
//	glr_ind = a / (1 + b*b);
//	gli_ind = -a*b / (1 + b*b);
//	pr = (1 - b*b) / (1 + b*b);
//	pi = 2 * b / (1 + b*b);
//
//	correctr = (1 + 2 * b*b + R*a + b*b*b*b + R*a*b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//	correcti = R*a*b*(1 + b*b) / ((1 + b*b + R*a)*(1 + b*b + R*a) + R*R*a*a*b*b);
//
//
//	if (mNode1 >= 0) {
//		g(mNode1, mNode1) = g(mNode1, mNode1) + glr;
//		g(compOffset + mNode1, compOffset + mNode1) = g(compOffset + mNode1, compOffset + mNode1) + glr;
//		g(compOffset + mNode1, mNode1) = g(compOffset + mNode1, mNode1) + gli;
//		g(mNode1, compOffset + mNode1) = g(mNode1, compOffset + mNode1) - gli;
//	}
//
//	if (mNode2 >= 0) {
//		g(mNode2, mNode2) = g(mNode2, mNode2) + glr;
//		g(compOffset + mNode2, compOffset + mNode2) = g(compOffset + mNode2, compOffset + mNode2) + glr;
//		g(compOffset + mNode2, mNode2) = g(compOffset + mNode2, mNode2) + gli;
//		g(mNode2, compOffset + mNode2) = g(mNode2, compOffset + mNode2) - gli;
//	}
//
//	if (mNode1 >= 0 && mNode2 >= 0) {
//		g(mNode1, mNode2) = g(mNode1, mNode2) - glr;
//		g(compOffset + mNode1, compOffset + mNode2) = g(compOffset + mNode1, compOffset + mNode2) - glr;
//		g(compOffset + mNode1, mNode2) = g(compOffset + mNode1, mNode2) - gli;
//		g(mNode1, compOffset + mNode2) = g(mNode1, compOffset + mNode2) + gli;
//
//		g(mNode2, mNode1) = g(mNode2, mNode1) - glr;
//		g(compOffset + mNode2, compOffset + mNode1) = g(compOffset + mNode2, compOffset + mNode1) - glr;
//		g(compOffset + mNode2, mNode1) = g(compOffset + mNode2, mNode1) - gli;
//		g(mNode2, compOffset + mNode1) = g(mNode2, compOffset + mNode1) + gli;
//	}
//
//
//}
//
//void RxLine::init(double om, double dt) {
//
//	// Initialize internal state
//	currr = 0;
//	curri = 0;
//	cureqr = 0;
//	cureqi = 0;
//	deltavr = 0;
//	deltavi = 0;
//
//	deltavr_ind = 0;
//	deltavi_ind = 0;
//	currr_ind = 0;
//	curri_ind = 0;
//	cureqr_ind = 0;
//	cureqi_ind = 0; 
//}
//
//
//void RxLine::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
//
//
//	// Initialize internal state
//	cureqr_ind = pr*currr_ind + pi*curri_ind + glr*deltavr_ind - gli*deltavi_ind;
//	cureqi_ind = -pi*currr_ind + pr*curri_ind + gli*deltavr_ind + glr*deltavi_ind;
//	
//	cureqr = cureqr_ind*correctr - cureqi_ind*correcti;
//	cureqi = cureqi_ind*correctr + correcti*cureqr_ind;
//
//	//cout << "cureq = " << cureq << endl;
//
//	if (mNode1 >= 0) {
//		j(mNode1, 0) = j(mNode1, 0) - cureqr;
//		j(compOffset + mNode1, 0) = j(compOffset + mNode1, 0) - cureqi;
//	}
//
//	if (mNode2 >= 0) {
//		j(mNode2, 0) = j(mNode2, 0) + cureqr;
//		j(compOffset + mNode2, 0) = j(compOffset + mNode2, 0) + cureqi;
//	}
//}
//
//void RxLine::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
//	double vposr, vnegr;
//	double vposi, vnegi;
//
//	// extract solution
//	if (mNode1 >= 0) {
//		vposr = vt(mNode1, 0);
//		vposi = vt(compOffset + mNode1, 0);
//	}
//	else {
//		vposr = 0;
//		vposi = 0;
//	}
//
//	if (mNode2 >= 0) {
//		vnegr = vt(mNode2, 0);
//		vnegi = vt(compOffset + mNode2, 0);
//	}
//	else {
//		vnegr = 0;
//		vnegi = 0;
//	}
//
//	deltavr = vposr - vnegr;
//	deltavi = vposi - vnegi;
//
//	currr = glr*deltavr - gli*deltavi + cureqr;
//	curri = gli*deltavr + glr*deltavi + cureqi;
//
//	deltavr_ind = vposr - resistance*currr - vnegr;
//	deltavi_ind = vposi - resistance*curri - vnegi;
//
//	currr_ind = glr_ind*deltavr_ind - gli_ind*deltavi_ind + cureqr_ind;
//	curri_ind = gli_ind*deltavr_ind + glr_ind*deltavi_ind + cureqi_ind;
//
//}


RxLine::RxLine(std::string name, int src, int dest, int node3, double resistance, double inductance) : BaseComponent(name, src, dest, node3) {

	this->resistance = resistance;
	this->conductance = 1.0 / resistance;
	this->inductance = inductance;


}

void RxLine::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {

	// Set diagonal entries
	if (mNode1 >= 0) {
		g(mNode1, mNode1) = g(mNode1, mNode1) + conductance;
		g(compOffset + mNode1, compOffset + mNode1) = g(compOffset + mNode1, compOffset + mNode1) + conductance;
	}

	if (mNode3 >= 0) {
		g(mNode3, mNode3) = g(mNode3, mNode3) + conductance;
		g(compOffset + mNode3, compOffset + mNode3) = g(compOffset + mNode3, compOffset + mNode3) + conductance;
	}

	// Set off diagonal entries
	if (mNode1 >= 0 && mNode3 >= 0) {
		g(mNode1, mNode3) = g(mNode1, mNode3) - conductance;
		g(compOffset + mNode1, compOffset + mNode3) = g(compOffset + mNode1, compOffset + mNode3) - conductance;

		g(mNode3, mNode1) = g(mNode3, mNode1) - conductance;
		g(compOffset + mNode3, compOffset + mNode1) = g(compOffset + mNode3, compOffset + mNode1) - conductance;
	}

	double a = dt / (2 * inductance);
	double b = dt*om / 2;

	glr = a / (1 + b*b);
	gli = -a*b / (1 + b*b);
	pr = (1 - b*b) / (1 + b*b);
	pi = 2 * b / (1 + b*b);


	if (mNode3 >= 0) {
		g(mNode3, mNode3) = g(mNode3, mNode3) + glr;
		g(compOffset + mNode3, compOffset + mNode3) = g(compOffset + mNode3, compOffset + mNode3) + glr;
		g(compOffset + mNode3, mNode3) = g(compOffset + mNode3, mNode3) + gli;
		g(mNode3, compOffset + mNode3) = g(mNode3, compOffset + mNode3) - gli;
	}

	if (mNode2 >= 0) {
		g(mNode2, mNode2) = g(mNode2, mNode2) + glr;
		g(compOffset + mNode2, compOffset + mNode2) = g(compOffset + mNode2, compOffset + mNode2) + glr;
		g(compOffset + mNode2, mNode2) = g(compOffset + mNode2, mNode2) + gli;
		g(mNode2, compOffset + mNode2) = g(mNode2, compOffset + mNode2) - gli;
	}

	if (mNode3 >= 0 && mNode2 >= 0) {
		g(mNode3, mNode2) = g(mNode3, mNode2) - glr;
		g(compOffset + mNode3, compOffset + mNode2) = g(compOffset + mNode3, compOffset + mNode2) - glr;
		g(compOffset + mNode3, mNode2) = g(compOffset + mNode3, mNode2) - gli;
		g(mNode3, compOffset + mNode2) = g(mNode3, compOffset + mNode2) + gli;

		g(mNode2, mNode3) = g(mNode2, mNode3) - glr;
		g(compOffset + mNode2, compOffset + mNode3) = g(compOffset + mNode2, compOffset + mNode3) - glr;
		g(compOffset + mNode2, mNode3) = g(compOffset + mNode2, mNode3) - gli;
		g(mNode2, compOffset + mNode3) = g(mNode2, compOffset + mNode3) + gli;
	}

}

void RxLine::init(double om, double dt) {

	// Initialize internal state
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}

void RxLine::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {


	// Initialize internal state
	cureqr = pr*currr + pi*curri + glr*deltavr - gli*deltavi;
	cureqi = -pi*currr + pr*curri + gli*deltavr + glr*deltavi;

	//cout << "cureq = " << cureq << endl;

	if (mNode3 >= 0) {
		j(mNode3, 0) = j(mNode3, 0) - cureqr;
		j(compOffset + mNode3, 0) = j(compOffset + mNode3, 0) - cureqi;
	}

	if (mNode2 >= 0) {
		j(mNode2, 0) = j(mNode2, 0) + cureqr;
		j(compOffset + mNode2, 0) = j(compOffset + mNode2, 0) + cureqi;
	}
}

void RxLine::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (mNode3 >= 0) {
		vposr = vt(mNode3, 0);
		vposi = vt(compOffset + mNode3, 0);
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

