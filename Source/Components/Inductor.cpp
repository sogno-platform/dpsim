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
			 
	if (node1 >= 0) {
		g(node1,node1) = g(node1,node1)+ glr;
		g(compOffset+node1,compOffset+node1) = g(compOffset+node1,compOffset+node1)+ glr;
		g(compOffset+node1,node1) = g(compOffset+node1,node1)+ gli;
		g(node1,compOffset+node1) = g(node1,compOffset+node1)- gli;
	}

	else if (node2 >= 0) {
		g(node2,node2) = g(node2,node2)+ glr;
		g(compOffset+node2,compOffset+node2) = g(compOffset+node2,compOffset+node2)+ glr;
		g(compOffset+node2,node2) = g(compOffset+node2,node2)+ gli;
		g(node2,compOffset+node2) = g(node2,compOffset+node2)- gli;
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


/// Initialize internal state
void Inductor::init(int compOffset, double om, double dt) {
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

	if (node1 >= 0) {
		j(node1, 0) = j(node1, 0) - cureqr;
		j(compOffset+node1, 0) = j(compOffset+node1, 0) - cureqi;
	}

	if (node2 >= 0)	{
		j(node2, 0) = j(node2, 0) + cureqr;
		j(compOffset+node2, 0) = j(compOffset+node2, 0) + cureqi;
	}
}


void Inductor::postStep(DPSMatrix& g, DPSMatrix& j, DPSMatrix& vt, int compOffset, double om, double dt, double t) {
	double vposr, vnegr;
	double vposi, vnegi;

	// extract solution
	if (node1 >= 0)	{
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

	currr =  glr*deltavr-gli*deltavi+cureqr;
	curri =  gli*deltavr+glr*deltavi+cureqi;

	//cout << "curr = " << curr << endl;
}
