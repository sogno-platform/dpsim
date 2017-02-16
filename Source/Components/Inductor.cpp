#include "Inductor.h"

Inductor::Inductor(std::string name, int src, int dest, double inductance) : BaseComponent(name, src, dest) {
	this->inductance = inductance;
}	
		
void Inductor::applySystemMatrixStamp(DPSMatrix& g, int compOffset, double om, double dt) {
	double a = 2.0*inductance / dt;
	double b = om*inductance;

	glr = a/(a*a+b*b);
	gli = -b/(a*a+b*b);
	pr = cos(2*atan(om/(2/dt)));
	pi = -sin(2*atan(om/(2/dt)));
			 
	if (mNode1 >= 0) {
		g(mNode1,mNode1) = g(mNode1,mNode1)+ glr;
		g(compOffset+mNode1,compOffset+mNode1) = g(compOffset+mNode1,compOffset+mNode1)+ glr;
		g(compOffset+mNode1,mNode1) = g(compOffset+mNode1,mNode1)+ gli;
		g(mNode1,compOffset+mNode1) = g(mNode1,compOffset+mNode1)- gli;
	}

	if (mNode2 >= 0) {
		g(mNode2,mNode2) = g(mNode2,mNode2)+ glr;
		g(compOffset+mNode2,compOffset+mNode2) = g(compOffset+mNode2,compOffset+mNode2)+ glr;
		g(compOffset+mNode2,mNode2) = g(compOffset+mNode2,mNode2)+ gli;
		g(mNode2,compOffset+mNode2) = g(mNode2,compOffset+mNode2)- gli;
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


/// Initialize internal state
void Inductor::init(double om, double dt) {
	currr = 0;
	curri = 0;
	cureqr = 0;
	cureqi = 0;
	deltavr = 0;
	deltavi = 0;
}


void Inductor::step(DPSMatrix& g, DPSMatrix& j, int compOffset, double om, double dt, double t) {
	// Initialize internal state
	cureqr =  glr*deltavr-gli*deltavi+pr*currr-pi*curri;
	cureqi =  gli*deltavr+glr*deltavi+pi*currr+pr*curri;

	//cout << "cureq = " << cureq << endl;

	if (mNode1 >= 0) {
		j(mNode1, 0) = j(mNode1, 0) - cureqr;
		j(compOffset+mNode1, 0) = j(compOffset+mNode1, 0) - cureqi;
	}

	if (mNode2 >= 0)	{
		j(mNode2, 0) = j(mNode2, 0) + cureqr;
		j(compOffset+mNode2, 0) = j(compOffset+mNode2, 0) + cureqi;
	}
}


void Inductor::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (mNode1 >= 0)	{
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

	currr =  glr*deltavr-gli*deltavi+cureqr;
	curri =  gli*deltavr+glr*deltavi+cureqi;

	//cout << "curr = " << curr << endl;
}
